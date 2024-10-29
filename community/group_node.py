
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from community_interfaces.msg import (
    PiSpeechComplete,
    PiSpeechRequest,
    PersonTextResult,
    PersonTextRequest,
    GroupInfo, 
    DeleteGptMessageId
)
from community_interfaces.srv import (
    RelationshipAction
)
import community.configuration as config

import os, yaml
import numpy as np

from community.group_convo_manager import GroupConvoManager

from ament_index_python.packages import get_package_share_directory


class GroupNode(Node):

    def __init__(self):

        super().__init__('group_node')

        # Get group_id from launch file
        self.declare_parameter('group_id', 0)
        self.group_id = self.get_parameter('group_id').get_parameter_value().integer_value

        # The max number of people in this group
        # TODO delete? num_members not used currently
        self.num_members = len(config.GROUP_PI_ASSIGNMENTS.get(self.group_id).get('pi_ids'))
        # The people currently in this group (id numbers) (this changes)
        self.group_members = []
        # Any new members added
        self.new_members = []
        # Any members left
        self.left_members = []
        # Variable for if last requested speech has been spoken by the pi
        self.last_speech_completed = True
        # Variable for if last person gpt text request has been recieved
        self.last_text_recieved = True
        # Seq for sending text requests
        self.text_seq = 0
        # Seq for sending speech requests
        self.speech_seq = 0
        # Seq for sending delete gpt message id
        self.delete_seq = 0
        # Seq for receiving group info
        self.group_info_seq = -1

        # Tick number for this group for the relationship manager, for latest text in the 'to speak' list
        self.relationships_tick_to_speak = 0
        # Tick number for this group for the relationship manager, for the most recently spoken text
        self.relationships_tick_spoken = 0

        # List of text from person GPTs, things TO SPEAK in FUTURE
        self.speak_list = []
        # List of text that HAS BEEN spoken by the RPis
        self.spoken_list = []

        # Initialise GroupConvoManager object
        self.group_convo_manager = GroupConvoManager()

        # Get the path to the `people.yaml` file
        package_share_dir = get_package_share_directory('community')
        people_path = os.path.join(package_share_dir, 'config_files', 'people.yaml')
        self.people_data = self.load_people(people_path)

        # Initialise publishers
        self.pi_speech_request_publisher = self.create_publisher(PiSpeechRequest, 'pi_speech_request', 10)
        self.delete_gpt_message_id_publisher = self.create_publisher(DeleteGptMessageId, 'delete_gpt_message_id', 10)
        self.person_text_request_publisher = self.create_publisher(PersonTextRequest, 'person_text_request', 10)
        # Timer callback
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback

        # Initialise subscribers
        self.person_text_result_subscription = self.create_subscription(
            PersonTextResult,
            'person_text_result', 
            self.person_text_result_callback, 
            10
        )
        self.pi_speech_complete_subscription = self.create_subscription(
            PiSpeechComplete,
            'pi_speech_complete', 
            self.pi_speech_complete_callback, 
            10
        )
        self.group_info_subscription = self.create_subscription(
            GroupInfo,
            'group_info', 
            self.group_info_callback, 
            10
        )
        # Prevent unused variable warnings
        self.pi_speech_complete_subscription
        self.group_info_subscription 

        # Create a client for the 'tick_get_relationship' service
        self.tick_get_relationship_client = self.create_client(RelationshipAction, 'tick_get_relationship')

        # Create a client for the 'rewind_relationship' service
        self.rewind_relationship_client = self.create_client(RelationshipAction, 'rewind_relationship')

        # Wait for service availability
        while not self.tick_get_relationship_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('tick_get_relationship service not available, waiting...')
        
        while not self.rewind_relationship_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('rewind_relationship service not available, waiting...')


    def load_people(self, file_path):
        """ Load people data from the YAML file. """
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data['people']

    def group_info_callback(self, msg):
        """
        Callback function for info on group assignment/members.
        """
        if msg.seq > self.group_info_seq:
            self.get_logger().debug('In group_info_callback')
            if msg.group_id == self.group_id:
                # Check for people who have left, if there were >0 people in the group previously
                if len(self.group_members) != 0:
                    for person_id in self.group_members:
                        if person_id not in msg.person_ids:
                            self.get_logger().info('Someone left the group')
                            self.group_members.remove(person_id)
                            self.get_logger().info(f'Updated group_members: {self.group_members}')
                            # Clear speech list as group makeup has changed
                            self.delete_gpt_message_id_and_rewind_relationship_tick()
                            self.speak_list = []
                            self.last_text_recieved = True
                            # Add one to text_seq if the speak_list has been reset so that 
                            # text results from requests sent before the reset are ignored
                            self.text_seq += 1
                # Check for new person added, if there >0 members of the group in the message
                if np.count_nonzero(msg.person_ids) != 0:
                    for person_id in msg.person_ids:
                        if person_id not in self.group_members and person_id != 0:
                            self.get_logger().info('Someone joined the group')
                            self.group_members.append(person_id)
                            self.get_logger().info(f'Updated group_members: {self.group_members}')
                            # Clear speech list as group makeup has changed
                            self.delete_gpt_message_id_and_rewind_relationship_tick()
                            self.speak_list = []
                            self.last_text_recieved = True
                            # Add one to text_seq if the speak_list has been reset so that 
                            # text results from requests sent before the reset are ignored
                            self.text_seq += 1
            self.group_info_seq = msg.seq

    def delete_gpt_message_id_and_rewind_relationship_tick(self):
        """
        Extract gpt_message_ids for each new instance of a person_id in self.speak_list.
        Publish a message to delete these gpt messages.
        Ask the RelationshipManagr to rewind to the last spoken tick.
        """
        # Only do anything if there are some unpublished messages in speak_list
        if len(self.speak_list) > 0:
            
            # Collect unique person_id entries from the right side of the list
            unique_person_ids = set()  # To store the unique person_ids
            result = []  # To store the results (person_id and gpt_message_id)

            for entry in self.speak_list:
                person_id = entry['person_id']
                if person_id not in unique_person_ids:
                    # Step 3: If it's a new person_id, save the person_id and gpt_message_id
                    unique_person_ids.add(person_id)
                    result.append({
                        'person_id': person_id,
                        'gpt_message_id': entry['gpt_message_id']
                    })

            # Publish a DeleteGptMessageId msg for each item in result list
            for item in result:
                msg = DeleteGptMessageId()
                msg.seq = self.delete_seq
                msg.group_id = self.group_id
                msg.person_id = item['person_id']
                msg.gpt_message_id = item['gpt_message_id']
                for i in range(5):
                    self.delete_gpt_message_id_publisher.publish(msg)
                self.delete_seq +=1

            # Only do if something has been spoken already
            if len(self.spoken_list) != 0:
                # Rewind relationships to tick in last item in spoken_list
                rewind_tick = None
                for num, _ in enumerate(self.spoken_list):
                    # Find the last item where the relationships were actually ticked
                    if self.spoken_list[-(num+1)]['relationship_ticked'] == True:
                        rewind_tick = self.spoken_list[-(num+1)]['relationship_tick']
                        break
                if rewind_tick != None:
                    success = self.call_rewind_relationship(self.group_id, rewind_tick, self.group_members)
                    if success != True:
                        self.get_logger().error("call_rewind_relationship failed!")


    def text_request_no_relationship(self, person_id, directed_id, event_id, message_type):
        msg = PersonTextRequest()
        msg.seq = self.text_seq
        msg.group_id = self.group_id
        msg.relationship_ticked = False
        msg.relationship_tick = 0
        msg.state_changed = False
        msg.from_state = "None"
        msg.to_state = "None"
        msg.action = "None" 
        msg.event_id = event_id
        msg.person_id = person_id
        msg.message_type = message_type
        msg.directed_id = directed_id 
        for i in range(5):
            self.person_text_request_publisher.publish(msg)
        self.get_logger().info('here5')
        self.last_text_recieved = False

    def text_request_with_relationship(self, person_id, directed_id, event_id, message_type):
        """
        Call the RelationshipManager service to tick on the relationship between two people,
        and return the current relationship state.
        
        :param person_a: The first person in the relationship.
        :param person_b: The second person in the relationship.
        :param group_id: The group that both people are in.
        :param group_members: All the people in the group (including person_a and person_b)
        
        :returns response.state_changed: bool, has the state changed?
        :returns from_state: what state we have changed from, if any
        :returns to_state: what state we have changed to, if any
        :returns action: What action to speak about, if any
        """
        # Create a request message
        request = RelationshipAction.Request()
        request.person_a = person_id
        request.person_b = directed_id
        request.group_id = self.group_id
        request.group_members = self.group_members
        self.get_logger().info("here7")

        # Send the request to the service and wait for the response
        future = self.tick_get_relationship_client.call_async(request) #TODO not working !!! -> make a ros_test_env and play with this. read documentation.
        # rclpy.spin_until_future_complete(self, future)
        future.add_done_callback(lambda future: self.handle_tick_get_relationship_response(future, event_id, person_id, message_type, directed_id))

    # Define the callback function to handle the response:
    def handle_tick_get_relationship_response(self, future, event_id, person_id, message_type, directed_id):
        self.get_logger().info("here8")
        try:
            response = future.result()
            if response:
                self.get_logger().info(f"Received response: state changed: {response.state_changed}, from: {response.from_state}, to: {response.to_state}, action: {response.action}, tick_id: {response.tick_id}")
                # Check if the request was successful
                if future.result() is not None:
                    response = future.result()
                    self.get_logger().info("here11")
                    msg = PersonTextRequest()
                    msg.seq = self.text_seq
                    msg.group_id = self.group_id
                    msg.relationship_ticked = True
                    msg.relationship_tick = response.tick_id
                    msg.state_changed = response.state_changed
                    msg.from_state = response.from_state
                    msg.to_state = response.to_state
                    msg.action = response.action
                    msg.transition_description = response.transition_description
                    msg.event_id = event_id
                    msg.person_id = person_id
                    msg.message_type = message_type
                    msg.directed_id = directed_id 
                    for i in range(5):
                        self.person_text_request_publisher.publish(msg)
                    self.get_logger().info('here5')
                    self.last_text_recieved = False
                else:
                    self.get_logger().error("Failed to call tick_get_relationship service")
                    
            else:
                self.get_logger().error("Received an empty response from tick_get_relationship service.")
        except Exception as e:
            self.get_logger().error(f"Service call failed with exception: {e}")

    def call_rewind_relationship(self, group_id, tick_id, group_members):
        """
        Call the RelationshipManager service to rewind the relationships for this group to a given tick.
        
        :param group_id: The id of this group.
        :param tick_id: The tick to rewind to.
        :param group_members: The people in this group.
        
        :returns: True or False, whether the rewind request succeeded or not.
        """
        # Create a request message
        request = RelationshipAction.Request()
        request.group_id = group_id
        request.tick_id = tick_id
        request.group_members = group_members

        # Send the request to the service and wait for the response
        future = self.rewind_relationship_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Check if the request was successful
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Rewind successful: {response.success}")
            return True
        else:
            self.get_logger().error("Failed to call rewind_relationship service")
            return False

    def person_text_result_callback(self, msg):
        """
        Callback for results from a person text request.
        """
        if msg.seq == self.text_seq and msg.group_id == self.group_id:
            self.get_logger().info('In person_text_result_callback')
            self.speak_list.append({
                'person_id' : msg.person_id,
                'pi_id' : msg.pi_id,
                'group_id' : msg.group_id,
                'people_in_group': msg.people_in_group,
                'text' : msg.text,
                'gpt_message_id' : msg.gpt_message_id,
                'directed_id' : msg.directed_id,
                'relationship_ticked' : msg.relationship_ticked,
                'relationship_tick' : msg.relationship_tick
            })
            self.last_text_recieved = True
            self.text_seq += 1

    def pi_speech_complete_callback(self, msg):
        """
        Callback for info that the pi has finished speaking a requested text.
        """
        if msg.seq == self.speech_seq and msg.group_id == self.group_id:
            self.get_logger().info('In pi_speech_complete_callback')
            if msg.complete == True:
                self.spoken_list.append({
                    'person_id' : msg.person_id,
                    'pi_id' : msg.pi_id,
                    'group_id' : msg.group_id,
                    'people_in_group': msg.people_in_group,
                    'text' : msg.text,
                    'gpt_message_id' : msg.gpt_message_id,
                    'directed_id' : msg.directed_id,
                    'relationship_ticked' : msg.relationship_ticked,
                    'relationship_tick' : msg.relationship_tick
                })
            self.last_speech_completed = True # Means that the pi acknowledged receipt, not necessarily that it was spoken out loud.
            self.speech_seq += 1

    def timer_callback(self):
        """
        Every timer_period seconds, check if a next text request or speech request is needed.
        If yes, request it.
        """
        # self.get_logger().info(str(self.last_speech_completed))
        # self.get_logger().info(str(self.last_text_recieved))
        # Check if new speech required (if last person's speech has been spoken).
        # Send a request to the Pi to SPEAK.
        if self.last_speech_completed == True and len(self.speak_list) != 0:
            self.get_logger().info('PUBLISHING SPEECH')
            # Use the FIRST item in speak_list
            text_dict = self.speak_list.pop(0)
            # Double check the person is still in the group
            if text_dict['person_id'] in self.group_members:
                msg = PiSpeechRequest()
                msg.seq = self.speech_seq
                voice_id = self.get_voice_id(text_dict['person_id'])
                msg.voice_id = voice_id
                self.get_logger().info(f'Voice_id here: {voice_id}')
                msg.person_id = text_dict['person_id']
                msg.pi_id = text_dict['pi_id']
                msg.group_id = text_dict['group_id']
                msg.people_in_group = text_dict['people_in_group']
                msg.text = text_dict['text']
                msg.gpt_message_id = text_dict['gpt_message_id']
                msg.directed_id = text_dict['directed_id']
                msg.relationship_ticked = text_dict['relationship_ticked']
                msg.relationship_tick = text_dict['relationship_tick']
                for i in range(5):
                    self.pi_speech_request_publisher.publish(msg)
                self.last_speech_completed = False
            else:
                self.get_logger().error("Person who should speak is not in group currently!")

        # If last text was recevied or the group members have just been changed
        # Send a a request to a person for TEXT
        # TODO ever a case where we send a person too many text requests?
        if self.last_text_recieved == True and len(self.group_members) > 0 and len(self.speak_list) < config.MAX_SPEAK_LIST_LEN:
            self.get_logger().info('here1')
            
            if len(self.speak_list) != 0:
                # Get last_speaker, second_last_speaker, and last_message_directed from speech list
                last_item = self.speak_list[-1]  # Get the last item in the list
                last_speaker = last_item['person_id']
                last_message_directed = last_item['directed_id']
                if len(self.speak_list) > 1:
                    second_last_item = self.speak_list[-2]
                    second_last_speaker = second_last_item['person_id']
                else:
                    second_last_speaker = 0
            elif len(self.spoken_list) != 0:
                # Group has reset in some way - get from spoken list.
                last_item = self.spoken_list[-1]  # Get the last item in the list
                last_speaker = last_item['person_id']
                last_message_directed = last_item['directed_id']
                if len(self.spoken_list) > 1:
                    second_last_item = self.spoken_list[-2]
                    second_last_speaker = second_last_item['person_id']
                else:
                    second_last_speaker = 0
            else:
                last_speaker = 0
                last_message_directed = 0
                second_last_speaker = 0
            person_id, message_type, directed_id, event_id = self.group_convo_manager.get_next(self.group_members, last_speaker, second_last_speaker, last_message_directed)
            if directed_id != 0:
                self.get_logger().info('here34')
                self.text_request_with_relationship(person_id, directed_id, event_id, message_type)
                # If the message is going to be directed at someone, tick the relationship manager and get back relationship info
                self.get_logger().info('here4') 
            else:
                self.text_request_no_relationship(person_id, directed_id, event_id, message_type)
            


    def get_voice_id(self, person_id):
        # Now initialize the person object using person attributes from config yaml file
        person_data = self.people_data.get(person_id, {})
        voice_id = person_data.get('voice_id', None)
        self.get_logger().info(f"Voice ID is: {voice_id}")
        if voice_id == None:
            self.get_logger().info("Error! Voice_id not found for this person_id")
        return str(voice_id)
            
            

def main(args=None):
    rclpy.init(args=args)

    group_node = GroupNode()

    rclpy.spin(group_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    group_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



    


    
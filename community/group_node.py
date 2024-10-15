
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
import community.configuration as config

import cv2, math, time, logging, pickle, random
import numpy as np

from community.group_convo_manager import GroupConvoManager


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

        # List of text from person GPTs, things TO SPEAK in FUTURE
        self.speech_list = []
        # List of text that HAS BEEN spoken by the RPis
        self.spoken_list = []

        # Initialise GroupConvoManager object
        self.group_convo_manager = GroupConvoManager()

        # TODO add RelationshipAction service call

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
                            self.get_logger().info(f'self.group_members {self.group_members}')
                            # Clear speech list as group makeup has changed
                            self.publish_delete_gpt_message_id()
                            self.speech_list = []
                            self.last_text_recieved = True
                            # Add one to text_seq if the speech_list has been reset so that 
                            # text results from requests sent before the reset are ignored
                            self.text_seq += 1
                # Check for new person added, if there >0 members of the group in the message
                if np.count_nonzero(msg.person_ids) != 0:
                    for person_id in msg.person_ids:
                        if person_id not in self.group_members and person_id != 0:
                            self.get_logger().info('Someone joined the group')
                            self.group_members.append(person_id)
                            self.get_logger().info(f'self.group_members {self.group_members}')
                            # Clear speech list as group makeup has changed
                            self.publish_delete_gpt_message_id()
                            self.speech_list = []
                            self.last_text_recieved = True
                            # Add one to text_seq if the speech_list has been reset so that 
                            # text results from requests sent before the reset are ignored
                            self.text_seq += 1
            self.group_info_seq = msg.seq

    def publish_delete_gpt_message_id(self):
        """
        Extract gpt_message_ids for each new instance of a person_id in self.speech_list
        """
        # Only do anything if there are some unpublished messages in speech_list
        if len(self.speech_list) > 0:
            
            # Collect unique person_id entries from the right side of the list
            unique_person_ids = set()  # To store the unique person_ids
            result = []  # To store the results (person_id and gpt_message_id)

            for entry in self.speech_list:
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

    def person_text_result_callback(self, msg):
        """
        Callback for results from a person text request.
        """
        if msg.seq == self.text_seq and msg.group_id == self.group_id:
            self.get_logger().info('In person_text_result_callback')
            self.speech_list.append({
                'person_id' : msg.person_id,
                'pi_id' : msg.pi_id,
                'group_id' : msg.group_id,
                'people_in_group': msg.people_in_group,
                'text' : msg.text,
                'gpt_message_id' : msg.gpt_message_id
                'directed_id' : msg.directed_id
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
                    'gpt_message_id' : msg.gpt_message_id
                    'directed_id' : msg.directed_id
                })
                self.last_speech_completed = True
                self.speech_seq += 1

    def timer_callback(self):
        """
        Every timer_period seconds, check if a next text request or speech is needed.
        If yes, request it.
        """
        # Check if new speech required (if last person's speech has been spoken).
        # Send a request to the Pi to SPEAK.
        if self.last_speech_completed == True and len(self.speech_list) != 0:
            self.get_logger().info('PUBLISHING SPEECH')
            # Use the FIRST item in speech_list
            text_dict = self.speech_list.pop(0)
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
                for i in range(5):
                    self.pi_speech_request_publisher.publish(msg)
                self.last_speech_completed = False

        # If last text was recevied or the group members have just been changed
        # Send a a request to a person for TEXT
        # TODO ever a case where we send a person too many text requests?
        if self.last_text_recieved == True and len(self.group_members) > 0 and len(self.speech_list) < config.MAX_SPEECH_LIST_LEN:
            self.get_logger().info('here1')
            msg = PersonTextRequest()
            msg.seq = self.text_seq
            msg.group_id = self.group_id
            if len(self.speech_list) != 0:
                # Get last_speaker and last_message_directed from speech list
                last_item = self.speech_list[-1]  # Get the last item in the list
                last_speaker = last_item['person_id']
                last_message_directed = last_item['directed_id']
            elif len(self.spoken_list) != 0:
                # Group has reset in some way - get from spoken list.
                last_item = self.spoken_list[-1]  # Get the last item in the list
                last_speaker = last_item['person_id']
                last_message_directed = last_item['directed_id']
            else:
                last_speaker = 0
                last_message_directed = 0
            msg.person_id, msg.message_type, msg.directed_id = self.group_convo_manager.get_next(self.group_members, last_speaker, last_message_directed)
            # TODO call relationship manager here !
            # Based on who is going to speak next, we can tick forward relationships by one.
            # Then the next relationship state can affect what is said next.  
            # Also TODO rewinding to ticks in the relationship manager... current relationship tick will need to be stored here as well.
            # Note if the relationship manager returns an action... the next to speak should NOT do an action so there is a 
            # chance to hear a response to the action.  
            for i in range(5):
                self.person_text_request_publisher.publish(msg)
            self.last_text_recieved = False


    def get_voice_id(self, person_id):
        voice_id = config.PERSON_INFO_DICT.get(person_id, {}).get('voice_id', None)
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



    


    
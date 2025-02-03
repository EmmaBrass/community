
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
from community.message_type import MessageType

from piper.voice import PiperVoice

import os, yaml, time, random, wave
import numpy as np

from pydub import AudioSegment
from pydub.generators import WhiteNoise, Sine

from community.group_convo_manager import GroupConvoManager
from community.helper_functions import HelperFunctions

from ament_index_python.packages import get_package_share_directory


class GroupNode(Node):

    def __init__(self):

        super().__init__('group_node')

        # Get group_id from launch file
        self.declare_parameter('group_id', 0)
        self.group_id = self.get_parameter('group_id').get_parameter_value().integer_value

        # The number of pis in this group
        self.num_members = len(config.GROUP_PI_ASSIGNMENTS.get(self.group_id).get('pi_ids'))
        # The people currently in this group (id numbers) (this changes)
        self.group_members = []
        # Simple pi-person dict updated from group_info: used for goodbye text
        self.person_pi_dict = None # for looking up pi from person id
        self.pi_person_dict = None # for looking up person from pi id
        # Flag for if a new member has been added
        self.new_member_flag = False
        # Flag for if a member has left
        self.left_member_flag = False
        # Variable for if last requested speech has been spoken by the pi
        self.last_speech_completed = True
        # Variable for indicating when in the process of compiling and sending new speech request
        self.creating_speech_request = False
        # Variable for if last person gpt text request has been recieved
        self.last_text_recieved = True
        # Variable for indicating when in the process of compiling and sending new text request
        self.creating_text_request = False
        # If very beginning of system, mention question
        self.first_question = True
        # Seq for sending text requests
        self.text_seq = 0
        # Seq for sending speech requests
        self.speech_seq = 1
        # Seq for sending delete gpt message id
        self.delete_seq = 0
        # Seq for receiving group info
        self.group_info_seq = -1
        # Simple lock to stop timer_callback from running when we are in group_info_callback
        self.group_info_lock = False

        # Arrays for chaos phase:
        self.group_pis = config.GROUP_PI_ASSIGNMENTS.get(self.group_id).get('pi_ids') # The pis listed in a constant order - use w/ self.person_pi_dict for tracking completed speech.
        self.chaos_last_speech_completed_dict = {pi_id: True for pi_id in self.group_pis}
        self.chaos_last_text_received_dict = {pi_id: True for pi_id in self.group_pis}
        self.chaos_text_seq_dict = {pi_id: 0 for pi_id in self.group_pis}
        self.chaos_speech_seq_dict = {pi_id: 0 for pi_id in self.group_pis}
        self.chaos_speak_list = {pi_id: [] for pi_id in self.group_pis}

        # For logging
        self.prev_last_speech_completed = True
        self.prev_last_text_recieved = True

        # Tick number for this group for the relationship manager, for latest text in the 'to speak' list
        self.relationships_tick_to_speak = 0
        # Tick number for this group for the relationship manager, for the most recently spoken text
        self.relationships_tick_spoken = 0

        # List of text from person GPTs, things TO SPEAK in FUTURE
        self.speak_list = []
        # List of text that HAS BEEN SENT to the RPi; has a COMPLETE field for if RPi spoke it successfully
        self.spoken_list = []

        # Some random goodbye phrases to say to fill space
        self.goodbye_list = [
            "Oh someone left, see you later, shame you could not stay.",
            "We have lost someone, this keeps happeneind, goodbye!",
            "A fellow group member has left us, that is sad.",
            "Ah, goodbye, friend! Come back soon.",
            "Hasta la vista baby",
            "Ah we have lost a group member! In a while crocodile",
            "Oh someone left, see you later alligator."
        ]

        # Initialise GroupConvoManager object
        self.group_convo_manager = GroupConvoManager()

        # Initialise question phase checker
        self.helper = HelperFunctions()

        # Place for static sounds files
        self.static_output_directory = "./static_sounds"
        os.makedirs(self.static_output_directory, exist_ok=True)

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
        TODO timer_callback cannot run whilst something is happening in here (and vice versa?)
        """
        
        if msg.seq > self.group_info_seq:
            self.get_logger().debug('In group_info_callback')
            if msg.group_id == self.group_id:
                self.group_info_lock = True
                # Update raw pi-person matching arrays (used for goodbye text)
                self.person_pi_dict = dict(zip(msg.person_ids, msg.pi_ids))
                self.pi_person_dict = {pi_id: person_id for person_id, pi_id in self.person_pi_dict.items()}
                # Check for people who have left, if there were >0 people in the group previously
                if len(self.group_members) != 0:
                    for person_id in self.group_members:
                        if person_id not in msg.person_ids:
                            self.get_logger().info('Someone left the group')
                            self.group_members.remove(person_id)
                            self.get_logger().info(f'Updated group_members: {self.group_members}')
                            # Rewind speak list as group makeup has changed, if not in chaos phase
                            question_phase = self.helper.get_question_phase()
                            if question_phase < config.CHAOS_QUESTION_PHASE:
                                self.rewind_speak(False, person_id)
                                self.last_text_recieved = True
                                # Add one to text_seq if the speak_list has been reset so that 
                                # text results from requests sent before the reset are ignored
                                self.text_seq += 1
                # Check for new person added, if there >0 members of the group in the message
                if np.count_nonzero(msg.person_ids) != 0:
                    for person_id in msg.person_ids:
                        if person_id not in self.group_members and person_id != 0:
                            # Flag that a new member has been added to slow down the timer_callback
                            # To ensure the person node has updated with new group is
                            self.new_member_flag = True
                            self.get_logger().info('Someone joined the group')
                            self.group_members.append(person_id)
                            self.get_logger().info(f'Updated group_members: {self.group_members}')
                            # Rewind speak list as group makeup has changed, if not in chaos phase
                            question_phase = self.helper.get_question_phase()
                            if question_phase < config.CHAOS_QUESTION_PHASE:
                                self.rewind_speak(True, person_id)
                                self.last_text_recieved = True
                                self.get_logger().info('Just set last_text_recieved to true')
                                # Add one to text_seq if the speak_list has been reset so that 
                                # text results from requests sent before the reset are ignored
                                self.text_seq += 1
                self.group_info_lock = False
                            
            self.group_info_seq = msg.seq

    def rewind_speak(self, joined, person_id):
        """
        Extract gpt_message_ids for each new instance of a person_id in self.speak_list.
        Publish a message to delete these gpt messages.
        Ask the RelationshipManagr to rewind to the last spoken tick.
        If someone joins, we rewind but leave the next two items
        If someone leaves, we rewind to but leave two UNLESS the person who left is in those two,
        in which case we rewind until the person who left is no longer in the list.

        :param joined: True if someone has joined the group, False if someone has left the group.
        :param person_id: The person who left or joined
        """
        # Only do anything if there are some unpublished messages in speak_list
        if len(self.speak_list) > 0:
            self.get_logger().info('Removing unspoken messages in speak list.')

            if joined == True:
                # If someone has joined, rewind all but last message in self.speak_list
                to_delete = self.speak_list[1:]
                self.speak_list = self.speak_list[:1]
            else:
                # If someone has left, rewind but leave two UNLESS person who left is in those two, then rewind further
                to_delete = self.speak_list
                new_speak_list = []
                for item in self.speak_list[:2]:
                    if item['person_id'] != person_id: # if person to speak is NOT the one who has just left, add to new_speak_list
                        new_speak_list.append(item)
                        del to_delete[0]
                    else:
                        break # stop building the new_speak_list
                self.speak_list = new_speak_list
                if self.speak_list == 0:
                    self.left_member_flag = True # so that someone will say 'goodbye' to fill the space.
                
            self.get_logger().info('Just reduced speak_list')
            
            # Collect unique person_id entries from the right side of the list, for deletion
            unique_person_ids = set()  # To store the unique person_ids
            result = []  # To store the results (person_id and gpt_message_id)

            for entry in to_delete:
                person_id = entry['person_id']
                if person_id not in unique_person_ids:
                    # If it's a new person_id, save the person_id and gpt_message_id
                    unique_person_ids.add(person_id)
                    result.append({
                        'person_id': person_id,
                        'gpt_message_id': entry['gpt_message_id']
                    })

            # Publish a DeleteGptMessageId msg for each item in result list
            self.get_logger().info('Publishing a DeleteGptMessageId msg for each item in result list.')
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
                self.get_logger().info('Rewinding relationships to tick in last item in spoken_list.')
                rewind_tick = None
                for num, _ in enumerate(self.spoken_list):
                    # Find the last item where the relationships were actually ticked
                    if self.spoken_list[-(num+1)]['relationship_ticked'] == True:
                        rewind_tick = self.spoken_list[-(num+1)]['relationship_tick']
                        break
                if rewind_tick != None:
                    self.call_rewind_relationship(self.group_id, rewind_tick, self.group_members)
                    

    def text_request_no_relationship(
            self, 
            seq,
            person_id, 
            directed_id, 
            event_id, 
            message_type, 
            question_id, 
            question_phase, 
            mention_question
        ):
        self.get_logger().info("In text_request_no_relationship")

        msg = PersonTextRequest()
        msg.seq = seq
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
        msg.question_id = question_id
        msg.question_phase = question_phase
        msg.mention_question = mention_question
        for i in range(5):
            self.person_text_request_publisher.publish(msg)
        self.last_text_recieved = False
        self.creating_text_request = False

    def text_request_with_relationship(
            self, 
            seq,
            person_id, 
            directed_id, 
            event_id, 
            message_type, 
            question_id, 
            question_phase, 
            mention_question
        ):
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
        self.get_logger().info("In text_request_with_relationship")

        # Create a request message
        request = RelationshipAction.Request()
        request.person_a = person_id
        request.person_b = directed_id
        request.group_id = self.group_id
        request.group_members = self.group_members

        # Send the request to the service and wait for the response
        future = self.tick_get_relationship_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future)
        future.add_done_callback(lambda future: self.handle_tick_get_relationship_response(future, 
                                                                                           seq,
                                                                                           event_id, 
                                                                                           person_id, 
                                                                                           message_type, 
                                                                                           directed_id, 
                                                                                           question_id, 
                                                                                           question_phase,
                                                                                           mention_question
                                                                                           ))

    # Define the callback function to handle the response:
    def handle_tick_get_relationship_response(
            self, 
            future, 
            seq,
            event_id, 
            person_id, 
            message_type, 
            directed_id, 
            question_id, 
            question_phase,
            mention_question
        ):
        try:
            response = future.result()
            if response:
                self.get_logger().info(f"Received response: state changed: {response.state_changed}, from: {response.from_state}, to: {response.to_state}, action: {response.action}, tick_id: {response.tick_id}")
                # Check if the request was successful
                if future.result() is not None:
                    response = future.result()
                    msg = PersonTextRequest()
                    msg.seq = seq
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
                    msg.question_id = question_id
                    msg.question_phase = question_phase
                    msg.mention_question = mention_question
                    for i in range(5):
                        self.person_text_request_publisher.publish(msg)
                    self.last_text_recieved = False
                    self.creating_text_request = False
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
        self.get_logger().info(f"Calling the RelationshipManager service to rewind the relationships for this group to a given tick.")
        # Create a request message
        request = RelationshipAction.Request()
        request.group_id = group_id
        request.tick_id = tick_id
        request.group_members = group_members

        # Send the request to the service and wait for the response
        future = self.rewind_relationship_client.call_async(request)
        future.add_done_callback(lambda future: self.call_rewind_relationship_callback(future))

    def call_rewind_relationship_callback(self, future):
        # Check if the request was successful
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Rewind successful: {response.success}")
            success = True
        else:
            self.get_logger().error("Failed to call rewind_relationship service")
            success = False
        
        if success != True:
            self.get_logger().error("call_rewind_relationship failed!")

    def person_text_result_callback(self, msg):
        """
        Callback for results from a person text request.
        """
        if msg.seq == self.text_seq and msg.group_id == self.group_id:
            self.get_logger().info('In person_text_result_callback')
            if self.helper.get_question_phase() < config.CHAOS_QUESTION_PHASE:
                self.speak_list.append({
                    'person_id' : msg.person_id,
                    'pi_id' : msg.pi_id,
                    'group_id' : msg.group_id,
                    'people_in_group': msg.people_in_group,
                    'message_type' : msg.message_type,
                    'text' : msg.text,
                    'gpt_message_id' : msg.gpt_message_id,
                    'directed_id' : msg.directed_id,
                    'relationship_ticked' : msg.relationship_ticked,
                    'relationship_tick' : msg.relationship_tick,
                    'mention_question' : msg.mention_question,
                    'question_id' : msg.question_id
                })
                self.get_logger().info('SPEAK LIST')
                self.get_logger().info(str(self.speak_list))
                self.last_text_recieved = True
                self.text_seq += 1
            else:
                self.chaos_speak_list[msg.pi].append({
                    'person_id' : msg.person_id,
                    'pi_id' : msg.pi_id,
                    'group_id' : msg.group_id,
                    'people_in_group': msg.people_in_group,
                    'message_type' : msg.message_type,
                    'text' : msg.text,
                    'gpt_message_id' : msg.gpt_message_id,
                    'directed_id' : msg.directed_id,
                    'relationship_ticked' : msg.relationship_ticked,
                    'relationship_tick' : msg.relationship_tick,
                    'mention_question' : msg.mention_question,
                    'question_id' : msg.question_id
                })
                self.get_logger().info('CHAOS SPEAK LIST')
                self.get_logger().info(str(self.chaos_speak_list[msg.pi]))
                self.chaos_last_text_received_dict[msg.pi] = True
                self.chaos_text_seq_dict[msg.pi] += 1

    def pi_speech_complete_callback(self, msg):
        """
        Callback for info that the pi has finished speaking a requested text.
        """
        if msg.seq == self.speech_seq and msg.group_id == self.group_id:
            self.get_logger().info('In pi_speech_complete_callback')
            if self.helper.get_question_phase() < config.CHAOS_QUESTION_PHASE:
                if msg.complete == True:
                    # find item in spoken list with the same gpt_message_id
                    for item in reversed(self.spoken_list):
                        if (item['gpt_message_id'] == msg.gpt_message_id and
                        item['person_id'] == msg.person_id):
                            # update 'completed' to be True
                            item['completed'] = True
                            break
                self.last_speech_completed = True # Means that the pi acknowledged receipt, not necessarily that it was spoken out loud.
                self.speech_seq += 1
            else:
                self.chaos_last_speech_completed_dict[msg.pi_id] = True
                self.chaos_speech_seq_dict[msg.pi_id] += 1

    def timer_callback(self):
        """
        Every timer_period seconds, check if a next text request or speech request is needed.
        If yes, request it.
        """

        if self.prev_last_speech_completed != self.last_speech_completed:
            self.get_logger().info("self.last_speech_completed")
            self.get_logger().info(str(self.last_speech_completed))
            self.prev_last_speech_completed = self.last_speech_completed
        if self.prev_last_text_recieved != self.last_text_recieved:
            self.get_logger().info("self.last_text_recieved")
            self.get_logger().info(str(self.last_text_recieved))
            self.prev_last_text_recieved = self.last_text_recieved

        # Text requests will be sent directly from the person nodes 
        # rather than here, if we are past the chaos question phase.
        question_phase = self.helper.get_question_phase()

        if question_phase < config.CHAOS_QUESTION_PHASE and self.group_info_lock == False:

            if self.last_speech_completed == True and self.creating_speech_request == False and len(self.speak_list) == 0 and self.left_member_flag == True:
                self.left_member_flag = False
                self.creating_speech_request = True
                self.get_logger().info('PUBLISHING GOODBYE SPEECH')
                # Choose a random group member.
                speaker = random.choice(self.group_members)
                voice_id = self.helper.get_voice_id(speaker)
                color = self.helper.get_color(speaker)
                # Convert text to .wav audio file bytes.
                text = random.choice(self.goodbye_list)
                audio_uint8 = self.helper.text_to_speech_bytes(text, voice_id, self.group_id)
                if speaker in self.group_members:
                    msg = PiSpeechRequest()
                    msg.seq = self.speech_seq
                    msg.voice_id = voice_id
                    msg.person_id = speaker
                    msg.pi_id = self.person_pi_dict[speaker]
                    msg.color = color
                    msg.group_id = self.group_id
                    msg.people_in_group = self.group_members
                    msg.message_type = MessageType.GOODBYE.value
                    msg.text = text
                    msg.gpt_message_id = 0
                    msg.directed_id = 0
                    msg.relationship_ticked = False
                    msg.relationship_tick = 0
                    msg.chaos_phase = False
                    msg.audio_data = audio_uint8
                    for i in range(5):
                        self.pi_speech_request_publisher.publish(msg)
                    self.last_speech_completed = False
                else:
                    self.get_logger().error("Person who should speak is not in group currently!")
                self.creating_speech_request = False

            # Check if new speech required (if last person's speech has been spoken).
            # Send a request to the Pi to SPEAK.
            if self.last_speech_completed == True and self.creating_speech_request == False and len(self.speak_list) != 0:
                self.creating_speech_request = True
                self.get_logger().info('PUBLISHING SPEECH')
                # Use the FIRST item in speak_list.
                text_dict = self.speak_list.pop(0)
                # Convert text to .wav audio file bytes.
                voice_id = self.helper.get_voice_id(text_dict['person_id'])
                color = self.helper.get_color(text_dict['person_id'])
                audio_uint8 = self.helper.text_to_speech_bytes(text_dict['text'], voice_id, self.group_id)
                # Move it to the spoken list.
                text_dict['completed'] = False
                self.spoken_list.append(text_dict)
                # Double check the person is still in the group.
                if text_dict['person_id'] in self.group_members:
                    msg = PiSpeechRequest()
                    msg.seq = self.speech_seq
                    msg.voice_id = voice_id
                    self.get_logger().info(f'Voice_id here: {voice_id}')
                    msg.person_id = text_dict['person_id']
                    msg.pi_id = text_dict['pi_id']
                    msg.color = color
                    msg.group_id = text_dict['group_id']
                    msg.people_in_group = text_dict['people_in_group']
                    msg.message_type = text_dict['message_type']
                    msg.text = text_dict['text']
                    msg.gpt_message_id = text_dict['gpt_message_id']
                    msg.directed_id = text_dict['directed_id']
                    msg.relationship_ticked = text_dict['relationship_ticked']
                    msg.relationship_tick = text_dict['relationship_tick']
                    msg.chaos_phase = False
                    msg.audio_data = audio_uint8
                    for i in range(5):
                        self.pi_speech_request_publisher.publish(msg)
                    self.last_speech_completed = False
                else:
                    self.get_logger().error("Person who should speak is not in group currently!")
                self.creating_speech_request = False


            if self.last_text_recieved == True and self.creating_text_request == False and len(self.group_members) > 0 and len(self.speak_list) < config.MAX_SPEAK_LIST_LEN:
                self.creating_text_request = True
                self.get_logger().info("REQUESTING TEXT")
                if self.new_member_flag:
                    # Sleep for a few secs to ensure the person node has registered new group_id
                    self.get_logger().info("Processing new member, sleeping for 2 sec.")
                    time.sleep(2)
                    self.new_member_flag = False
                    self.creating_text_request = False
                    return
                self.get_logger().info("LEN SPEAK LIST")
                self.get_logger().info(str(len(self.speak_list)))
                if len(self.speak_list) != 0:
                    # Get last_speaker, second_last_speaker, and last_message_directed from speech list
                    last_item = self.speak_list[-1]  # Get the last item in the list
                    last_speaker = last_item['person_id']
                    last_message_directed = last_item['directed_id']
                    last_question_id = last_item['question_id']
                    if len(self.speak_list) > 1:
                        second_last_item = self.speak_list[-2]
                        second_last_speaker = second_last_item['person_id']
                    else:
                        second_last_speaker = 0
                elif len(self.spoken_list) != 0:
                    self.get_logger().info("SPOKEN LIST")
                    self.get_logger().info(str(self.spoken_list))
                    # Group has reset in some way - get from spoken list.
                    last_item = self.spoken_list[-1]  # Get the last item in the list
                    last_speaker = last_item['person_id']
                    last_message_directed = last_item['directed_id']
                    last_question_id = last_item['question_id']
                    if len(self.spoken_list) > 1:
                        second_last_item = self.spoken_list[-2]
                        second_last_speaker = second_last_item['person_id']
                    else:
                        second_last_speaker = 0
                else:
                    last_speaker = 0
                    last_message_directed = 0
                    second_last_speaker = 0
                    last_question_id = 0
                self.get_logger().info("LAST SPEAKER")
                self.get_logger().info(str(last_speaker))
                self.get_logger().info("2nd LAST SPEAKER")
                self.get_logger().info(str(second_last_speaker))
                person_id, message_type, directed_id, event_id, question_id, question_phase = self.group_convo_manager.get_next(
                    self.group_members, 
                    last_speaker, 
                    second_last_speaker, 
                    last_question_id,
                    last_message_directed
                )
                if message_type == MessageType.SWITCH.value or message_type == MessageType.ALONE.value:
                    mention_question = self.check_last_question_mention(person_id)
                else:
                    mention_question = False
                
                self.get_logger().info("person id")
                self.get_logger().info(str(person_id))
                self.get_logger().info("directed id")
                self.get_logger().info(str(directed_id))
                self.get_logger().info("question id")
                self.get_logger().info(str(question_id))
                self.get_logger().info("Completed group_convo_manager")
                self.get_logger().info("message type")
                self.get_logger().info(str(message_type))
                if directed_id != 0 and config.RELATIONSHIPS == True: 
                    # Relationships must be turned on in config file to go here
                    self.text_request_with_relationship(
                        self.text_seq,
                        person_id, 
                        directed_id, 
                        event_id, 
                        message_type, 
                        question_id, 
                        question_phase, 
                        mention_question
                    )
                    # If the message is going to be directed at someone, tick the relationship manager and get back relationship info
                else:
                    self.text_request_no_relationship(
                        self.text_seq,
                        person_id, 
                        directed_id, 
                        event_id, 
                        message_type, 
                        question_id, 
                        question_phase, 
                        mention_question
                    )

        elif question_phase >= config.CHAOS_QUESTION_PHASE and self.group_info_lock == False:

            if question_phase == config.CHAOS_QUESTION_PHASE:

                # If chaos question phase: still need to request & receive text AND send audio files from speak list
                for pi in self.group_pis:

                    # Request text
                    if self.chaos_last_text_received_dict[pi] == True:
                        self.chaos_last_text_received_dict[pi] = False
                        # Get current person_id for the pi
                        person_id = self.pi_person_dict[pi]
                        self.text_request_no_relationship(
                            seq = self.chaos_text_seq_dict[pi],
                            person_id = person_id, 
                            directed_id = 0, 
                            event_id = 0, 
                            message_type = MessageType.OPEN.value, 
                            question_id = person_id, 
                            question_phase = question_phase, 
                            mention_question = False
                        )

                    # Request speech
                    if self.chaos_last_speech_completed_dict[pi] == True:
                        # Get current person_id for the pi
                        person_id = self.pi_person_dict[pi]
                        voice_id = self.helper.get_voice_id(person_id)
                        color = self.helper.get_color(person_id)
                        # Use the FIRST item in speak_list.
                        text_dict = self.chaos_speak_list[pi].pop(0)
                        audio_uint8 = self.helper.text_to_speech_bytes(text_dict['text'], voice_id, person_id)
                        # Double check the person is still in the group.
                        if text_dict['person_id'] in self.group_members:
                            self.chaos_last_speech_completed_dict[pi] = False
                            msg = PiSpeechRequest()
                            msg.seq = self.chaos_speech_seq_dict[pi]
                            msg.voice_id = voice_id
                            msg.person_id = text_dict['person_id']
                            msg.pi_id = text_dict['pi_id']
                            msg.color = color
                            msg.group_id = text_dict['group_id']
                            msg.people_in_group = text_dict['people_in_group']
                            msg.message_type = text_dict['message_type']
                            msg.text = text_dict['text']
                            msg.gpt_message_id = text_dict['gpt_message_id']
                            msg.directed_id = text_dict['directed_id']
                            msg.relationship_ticked = text_dict['relationship_ticked']
                            msg.relationship_tick = text_dict['relationship_tick']
                            msg.chaos_phase = False
                            msg.audio_data = audio_uint8
                            for i in range(5):
                                self.pi_speech_request_publisher.publish(msg)

            # If static phase, need to request just random static from each.  
            if question_phase == config.STATIC_QUESTION_PHASE:
                for pi in self.group_pis:
                    if self.chaos_last_speech_completed_dict[pi] == True:
                        self.chaos_last_speech_completed_dict[pi] = False
                        # Get current person_id for the pi
                        person_id = self.pi_person_dict[pi]
                        voice_id = self.helper.get_voice_id(person_id)
                        color = self.helper.get_color(person_id)
                        audio_uint8 = self.generate_static_sounds(output_dir=self.static_output_directory, duration=6)
                        msg = PiSpeechRequest()
                        msg.seq = self.chaos_speech_seq_dict[pi]
                        msg.voice_id = voice_id
                        msg.person_id = person_id
                        msg.pi_id = pi
                        msg.color = color
                        msg.group_id = self.group_id
                        msg.people_in_group = self.group_members
                        msg.message_type = MessageType.OPEN.value
                        msg.text = "Static"
                        msg.gpt_message_id = 0
                        msg.directed_id = 0
                        msg.relationship_ticked = False
                        msg.relationship_tick = 0
                        msg.chaos_phase = True
                        msg.audio_data = audio_uint8

    def check_last_question_mention(self, person_id):
        """
        Check if this person explicitly mentioned their question recently.
        """
        mention_question = False

        if len(self.spoken_list) > 0:
            reversed_spoken_list = self.spoken_list[::-1]
            len_from_end = -1
            for num, item in enumerate(reversed_spoken_list):
                # Find the last item where this person mentioned their question
                if item['mention_question'] == True and item['person_id'] == person_id:
                    len_from_end = num
                    break
            if len_from_end > config.MIN_QUESTION_MENTION or len_from_end == -1: # If -1, then their question has not been mentioned at all in spoken list for this group.
                mention_question = True
        if self.first_question == True:
            mention_question = True
            self.first_question = False
 
        return mention_question
    
    def generate_static_sounds(self, output_dir, duration):
        """
        Generate varied electronic static audio files with white noise and modulation.

        :param output_dir: Directory to save the generated audio files.
        :param duration: Duration of each audio file in seconds.

        :returns: The audi data as a series of bytes.
        """
        # Generate base white noise
        noise = WhiteNoise().to_audio_segment(duration * 1000)  # Duration in milliseconds

        # Add random amplitude modulation (low-frequency sine wave)
        mod_freq = np.random.uniform(0.1, 5)  # Modulation frequency in Hz
        mod_depth = np.random.uniform(-15, -5)  # Modulation depth in dB
        sine_wave = Sine(mod_freq).to_audio_segment(duration * 1000).apply_gain(mod_depth)
        modulated_noise = noise.overlay(sine_wave)

        # Apply random low-pass and high-pass filters
        low_cutoff = np.random.uniform(500, 5000)  # Low-pass cutoff frequency
        high_cutoff = np.random.uniform(50, 400)   # High-pass cutoff frequency
        filtered_noise = modulated_noise.low_pass_filter(low_cutoff).high_pass_filter(high_cutoff)

        # Add slight random gain for variation
        final_noise = filtered_noise.apply_gain(np.random.uniform(-5, 10))

        # Export to file
        wav_file = f"{output_dir}/static_sound.wav"
        final_noise.export(wav_file, format="wav")
        print(f"Generated: {wav_file}")

        # Convert audio to bytes
        with open(wav_file, 'rb') as f:
            audio_data = f.read()
            audio_uint8 = list(audio_data)
            
        return audio_uint8
                
            

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



    


    
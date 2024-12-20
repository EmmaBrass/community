# Subscribe to faces_info topic
# Keep track of what system state we are in
# Publish state to system_state topic

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from community.message_type import MessageType
from community_interfaces.msg import (
    PiSpeechRequest,
    PiSpeechComplete,
    PersonTextRequest,
    PersonTextResult,
    GroupInfo,
    DeleteGptMessageId
)
import community.configuration as config
from community.person_llm import PersonLLM
from community.prompt_manager import PromptManager
from community.question_phase import GetQuestionPhase

import cv2, math, time, logging, pickle
import numpy as np
import yaml, os

from ament_index_python.packages import get_package_share_directory


class PersonNode(Node):

    def __init__(self):

        super().__init__('person_node')

        # Get person_id from launch file -> long multi-digit number of RFID card
        self.declare_parameter('person_id', 0)
        self.person_id = self.get_parameter('person_id').get_parameter_value().integer_value

        # Get the path to the `people.yaml` file
        package_share_dir = get_package_share_directory('community')
        people_path = os.path.join(package_share_dir, 'config_files', 'people.yaml')
        self.people_data = self.load_people(people_path)

        # Now initialize the person object using person attributes from config yaml file
        person_data = self.people_data.get(self.person_id, {})
        # Get voice id
        self.voice_id = str(person_data.get('voice_id', None))
        self.get_logger().info(f"Voice ID is: {self.voice_id}")
        if self.voice_id == None:
            self.get_logger().info("Error! Voice_id not found for this person_id")
        # Get color
        self.color = person_data.get('color', None)
        self.get_logger().info(f"Color is: {self.color}")
        if self.color == None:
            self.get_logger().info("Error! Color not found for this person_id")

        self.person = PersonLLM(
            person_id = self.person_id,
            name = person_data.get('name', "Person not found."),
            gender = person_data.get('gender', "Person not found."),
            age = person_data.get('age', "Person not found."),
            question = person_data.get('question', "Person not found."),
            history = person_data.get('history', "Person not found."),
            relationships = person_data.get('relationships', "Person not found."),
            personality = person_data.get('relationships', "Person not found.")
        )

        self.group_id = None # will change
        self.pi_id = None # will change
        self.group_members = [] # People in the group EXCLUDING this person
        # Seq list for receiving text requests - one item for each group
        self.text_seq = [-1]*config.NUM_GROUPS
        # Seq for receiving group info
        self.group_info_seq = -1
        # Seq list for receiving speech updates - one item for each group
        self.speech_seq = [-1]*config.NUM_GROUPS
        # Seq for deletie message id
        self.delete_seq = [-1]*config.NUM_GROUPS

        # For tracking speech from Pi for chaos phase
        self.last_chaos_completed = True
        # For chaos phase, checking were we are in request
        self.creating_speech_request = False
        # List of things to say in chaos phase
        self.speak_list = []
        # Seq for chaos phase
        self.chaos_seq = 1

        # Initialise prompt manager
        self.prompt_manager = PromptManager(self.person_id)

        # Initialise question phase checker
        self.question_phase = GetQuestionPhase()

        # Initialise publishers
        self.pi_speech_request_publisher = self.create_publisher(PiSpeechRequest, 'pi_speech_request', 10)
        # Timer callback
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback
        
        # Initialise publishers
        self.person_text_result_publisher = self.create_publisher(
            PersonTextResult, 
            'person_text_result', 
            10
        )
        # Initialise subscribers
        self.person_text_request_subscription = self.create_subscription(
            PersonTextRequest,
            'person_text_request', 
            self.person_text_request_callback, 
            10
        )
        self.person_text_result_subscription = self.create_subscription(
            PersonTextResult,
            'person_text_result', 
            self.person_text_result_callback, 
            10
        )
        self.group_info_subscription = self.create_subscription(
            GroupInfo,
            'group_info', 
            self.group_info_callback, 
            10
        )
        self.pi_speech_complete_subscription = self.create_subscription(
            PiSpeechComplete,
            'pi_speech_complete', 
            self.pi_speech_complete_callback, 
            10
        )
        self.delete_gpt_message_id_subscription = self.create_subscription(
            DeleteGptMessageId,
            'delete_gpt_message_id', 
            self.delete_gpt_message_id_callback, 
            10
        )
        # Prevent unused variable warnings
        self.person_text_request_subscription 
        self.person_text_result_subscription
        self.group_info_subscription 
        self.delete_gpt_message_id_subscription

    def load_people(self, file_path):
        """ Load people data from the YAML file. """
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data['people']

    def delete_gpt_message_id_callback(self, msg):
        """
        Delete messages on the GPT thread including and after 
        the message with the ID in the given ROS msg.
        """
        if msg.seq > self.delete_seq[msg.group_id-1] and msg.person_id == self.person_id:
            self.person.delete_messages_from_id(msg.gpt_message_id)
            self.get_logger().info("Successfully deleted unspoken gpt messages.")
            self.delete_seq[msg.group_id-1] = msg.seq


    def person_text_request_callback(self, msg):
        """
        Callback function for requesting text from the GPT for this person.
        """
        if msg.person_id == self.person_id \
            and msg.seq > self.text_seq[msg.group_id-1]:
            # and msg.group_id == self.group_id \
            self.get_logger().info('In person_text_request_callback')
            prompt_details = self.prompt_manager.get_prompt_details(
                msg.message_type, 
                msg.directed_id, 
                msg.event_id, 
                msg.state_changed, 
                msg.from_state, 
                msg.to_state, 
                msg.action, 
                msg.transition_description,
                msg.question_id,
                msg.question_phase,
                msg.mention_question
            )
            self.get_logger().info('////////////////////////////prompt_details')
            self.get_logger().info(str(prompt_details))
            # TODO a double check that the directed_to is actually in the group?
            text, gpt_message_id = self.person.person_speaks(
                self.person_id,
                self.group_id,
                self.group_members, # Members of the group EXCLUDING the person who will talk.
                prompt_details
            )
            self.get_logger().info('GPT request complete')
            self.get_logger().info(f'Text from GPT: {text}')
            self.get_logger().info(f'Message ID from GPT: {gpt_message_id}')
            self.person_text_result_pub(
                msg.seq, 
                text, 
                msg.message_type,
                gpt_message_id, 
                msg.directed_id, 
                msg.relationship_ticked, 
                msg.relationship_tick,
                msg.mention_question
            )
            self.text_seq[msg.group_id-1] = msg.seq

    def person_text_result_pub(self, 
                               seq: int, 
                               text: str, 
                               message_type: int,
                               gpt_message_id: int, 
                               directed_id: int, 
                               relationship_ticked: bool, 
                               relationship_tick: int, 
                               mention_question: bool
                               ):
        """
        Publish text to the person_text_result topic.

        :param seq: The seq id.
        :param text: The text returned from the GPT.
        """
        self.get_logger().info('In person_text_result_pub')
        msg = PersonTextResult()
        msg.seq = seq
        msg.person_id = self.person_id
        msg.pi_id = self.pi_id
        msg.group_id = self.group_id
        msg.people_in_group = self.group_members
        msg.message_type = message_type
        msg.text = text
        msg.gpt_message_id = gpt_message_id
        msg.directed_id = directed_id #TODO relationship tick and ticked ????
        msg.relationship_ticked = relationship_ticked
        msg.relationship_tick = relationship_tick
        msg.mention_question = mention_question
        for i in range(5):
            self.person_text_result_publisher.publish(msg)

    def group_info_callback(self, msg):
        """
        Callback function for receving information about group members.
        """
        if msg.seq > self.group_info_seq:
            self.get_logger().debug('In group_info_callback')
            if self.person_id in msg.person_ids:
                # Find the others in the group from the msg
                others_in_group = [person for person in msg.person_ids if person != self.person_id and person != 0]
                # Get the assigned pi for this person
                assigned_pi = msg.pi_ids[msg.person_ids.index(self.person_id)]

                # Check if the person has been moved to a different pi
                if assigned_pi != self.pi_id:
                    self.get_logger().info('This person has been placed on a new pi.')
                    # They have been moved
                    # Update the pi id
                    self.pi_id = assigned_pi

                # If they are in the same group as before
                if msg.group_id == self.group_id:
                    # Check if a person left the group, to send to the gpt as metadata
                    for person in self.group_members:
                        if person not in others_in_group:
                            self.get_logger().info('A person left the group')
                            self.group_members.remove(person)
                            # Tell the GPT about the member who left the group
                            self.person.member_left_group(person)
                    # Check for new person added, to send to the gpt as metadata
                    for person in others_in_group:
                        if person not in self.group_members: 
                            self.get_logger().info('A person joined the group')
                            self.group_members.append(person)
                            # Tell the GPT about the new group member
                            self.person.member_joined_group(person)
                # If they are in a different group from before
                elif msg.group_id != self.group_id:
                    self.get_logger().info('This person has joined a new group.')
                    self.group_id = msg.group_id
                    self.group_members = others_in_group
                    # Tell the GPT about the new group
                    self.person.new_group(self.group_id, self.group_members)
            self.group_info_seq = msg.seq

    def person_text_result_callback(self, msg):
        """
        Looks at incoming person_text_result data and tells the GPT about it if 
        this person is in the group (but they are not the speaker).
        """
        if (msg.person_id != self.person_id) and \
            (self.person_id in msg.people_in_group) and \
            (self.group_id == msg.group_id) and \
            (msg.seq > self.speech_seq[msg.group_id-1]):
            self.get_logger().info('In person_text_result_callback')
            self.person.other_member_text(
                msg.person_id, 
                msg.group_id, 
                msg.people_in_group, # This is everyone in the group EXCLUDING the speaker
                msg.text
                # msg.directed_id # Who the message was directed at
            )
            self.speech_seq[msg.group_id-1] = msg.seq

    def pi_speech_complete_callback(self, msg):
        """
        Callback for info that the pi has finished speaking a requested text.
        Only used once in chaos question phase.
        """
        if msg.seq == self.chaos_seq and msg.group_id == self.group_id and self.question_phase.get_question_phase() >= config.CHAOS_QUESTION_PHASE:
            self.get_logger().info('In pi_speech_complete_callback')
            self.last_chaos_completed = True
            self.chaos_seq += 1

    def timer_callback(self):
        """
        For when past the chaos question phase, 
        pi speech requests will be managed directly from the person nodes.
        """
        # Text requests will be sent directly from here rather than the group nodes,
        # IF we are past the chaos question phase only.
        question_phase = self.question_phase.get_question_phase()
        if question_phase >= config.CHAOS_QUESTION_PHASE:

            if self.last_chaos_completed == True and self.creating_speech_request == False and len(self.speak_list) != 0:
                self.creating_speech_request = True
                self.get_logger().info('PUBLISHING SPEECH')
                # Use the FIRST item in speak_list
                text_dict = self.speak_list.pop(0)
                # Double check the person is still in the group
                if text_dict['person_id'] in self.group_members:
                    msg = PiSpeechRequest()
                    msg.seq = self.chaos_seq
                    msg.voice_id = self.voice_id
                    self.get_logger().info(f'Voice_id here: {self.voice_id}')
                    msg.person_id = text_dict['person_id']
                    msg.pi_id = text_dict['pi_id']
                    msg.color = self.color
                    msg.group_id = text_dict['group_id']
                    msg.people_in_group = text_dict['people_in_group']
                    msg.message_type = text_dict['message_type']
                    msg.text = text_dict['text']
                    msg.gpt_message_id = text_dict['gpt_message_id']
                    msg.directed_id = text_dict['directed_id']
                    msg.relationship_ticked = text_dict['relationship_ticked']
                    msg.relationship_tick = text_dict['relationship_tick']
                    msg.chaos_phase = True
                    for i in range(5):
                        self.pi_speech_request_publisher.publish(msg)
                    self.last_chaos_completed = False
                    self.creating_speech_request = False
                else:
                    self.get_logger().error("Person who should speak is not in group currently!")

            if len(self.group_members) > 0 and len(self.speak_list) < config.MAX_SPEAK_LIST_LEN:
                prompt_details = self.prompt_manager.get_prompt_details(
                    message_type=MessageType.OPEN.value,
                    directed_id=0, 
                    event_id=0, 
                    state_changed=False, 
                    from_state="None", 
                    to_state="None", 
                    action="None", 
                    transition_description="None",
                    question_id=self.person_id,
                    question_phase=question_phase,
                    mention_question=False
                )
                self.get_logger().info('////////////////////////////prompt_details')
                self.get_logger().info(str(prompt_details))
                # TODO a double check that the directed_to is actually in the group?
                text, gpt_message_id = self.person.person_speaks(
                    self.person_id,
                    self.group_id,
                    self.group_members, # Members of the group EXCLUDING the person who will talk.
                    prompt_details
                )
                self.get_logger().info('GPT request complete')
                self.get_logger().info(f'Text from GPT: {text}')
                self.get_logger().info(f'Message ID from GPT: {gpt_message_id}')
                self.speak_list.append({
                    'person_id' : self.person_id,
                    'pi_id' : self.pi_id,
                    'group_id' : self.group_id,
                    'people_in_group': self.group_members,
                    'message_type' : MessageType.OPEN.value,
                    'text' : text,
                    'gpt_message_id' : gpt_message_id,
                    'directed_id' : msg.directed_id,
                    'relationship_ticked' : msg.relationship_ticked,
                    'relationship_tick' : msg.relationship_tick,
                    'mention_question' : msg.mention_question
                })


def main(args=None):
    rclpy.init(args=args)

    person_node = PersonNode()

    rclpy.spin(person_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    person_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


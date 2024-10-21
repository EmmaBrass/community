# Subscribe to faces_info topic
# Keep track of what system state we are in
# Publish state to system_state topic

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from community_interfaces.msg import (
    PiSpeechRequest,
    PersonTextRequest,
    PersonTextResult,
    GroupInfo,
    DeleteGptMessageId
)
import community.configuration as config
from community.person_llm import PersonLLM
from community.prompt_manager import PromptManager

import cv2, math, time, logging, pickle
import numpy as np
import yaml


class PersonNode(Node):

    def __init__(self):

        super().__init__('person_node')

        # Get person_id from launch file -> long multi-digit number of RFID card
        self.declare_parameter('person_id', 0)
        self.person_id = self.get_parameter('person_id').get_parameter_value().integer_value

        # Load the YAML file
        with open('people.yaml', 'r') as file: #TODO check yaml path
            person_info_dict = yaml.safe_load(file)

        # Now initialize the person object using person attributes from config yaml file
        person_data = person_info_dict.get(self.person_id, {})
        self.person = PersonLLM(
            person_id = self.person_id,
            name = person_data.get('name', "Person not found."),
            gender = person_data.get('gender', "Person not found."),
            age = person_data.get('age', "Person not found."),
            openness = person_data.get('openness', "Person not found."),
            conscientiousness = person_data.get('conscientiousness', "Person not found."),
            neuroticism = person_data.get('neuroticism', "Person not found."),
            agreeableness = person_data.get('agreeableness', "Person not found."),
            extraversion = person_data.get('extraversion', "Person not found."),
            history = person_data.get('history', "Person not found."),
            relationships = person_data.get('relationships', "Person not found."),
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

        # Initialise prompt manager
        self.prompt_manager = PromptManager()
        
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
            and msg.group_id == self.group_id \
            and msg.seq > self.text_seq[msg.group_id-1]:
            self.get_logger().info('In person_text_request_callback')
            prompt_details = self.prompt_manager(msg.message_type, msg.directed_id, msg.state_changed, msg.from_state, msg.to_state, msg.action)
            # TODO a double check that the directed_to is actually in the group?
            # TODO then prompt manager called!  To get extra parameters to pass to the person_llm
            # Decide on the mood/topic/ other instructions for the GPT.
            text, gpt_message_id = self.person.person_speaks(
                self.person_id,
                self.group_id,
                self.group_members, # Members of the group EXCLUDING the person who will talk.
                prompt_details
            )
            self.get_logger().info(f'Text from GPT!: {text}')
            self.get_logger().info(f'Message ID from GPT!: {gpt_message_id}')
            self.person_text_result_pub(msg.seq, text, gpt_message_id, msg.directed_id)
            self.text_seq[msg.group_id-1] = msg.seq

    def person_text_result_pub(self, seq, text, gpt_message_id, directed_id):
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
        msg.text = text
        msg.gpt_message_id = gpt_message_id
        msg.directed_id = directed_id
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
                others_in_group = [person for person in msg.person_ids if person != self.person_id]
                # Get the assigned pi for this person
                assigned_pi = msg.pi_ids[msg.person_ids.index(self.person_id)]
                # Check if the person has been moved to a different pi
                if assigned_pi != self.pi_id:
                    self.get_logger().info('This person has moved to a different pi')
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
                msg.text,
                msg.directed_id # Who the message was directed at
            )
            self.speech_seq[msg.group_id-1] = msg.seq


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


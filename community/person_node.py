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
    GroupInfo
)
import community.configuration as config
from community.person import Person

import cv2, math, time, logging, pickle
import numpy as np


class PersonNode(Node):

    def __init__(self):

        super().__init__('person_node')

        # Get person_id from launch file -> long multi-digit number of RFID card
        self.declare_parameter('person_id', 0)
        self.person_id = self.get_parameter('person_id').get_parameter_value().integer_value

        # Make person object using person attributes from config file
        self.person = Person(
            person_id = self.person_id,
            name = config.PERSON_INFO_DICT.get(self.person_id, {}).get('name', "Person not found."), 
            gender = config.PERSON_INFO_DICT.get(self.person_id, {}).get('gender', "Person not found."), 
            age = config.PERSON_INFO_DICT.get(self.person_id, {}).get('age', "Person not found."), 
            openness = config.PERSON_INFO_DICT.get(self.person_id, {}).get('openness', "Person not found."),  
            conscientiousness = config.PERSON_INFO_DICT.get(self.person_id, {}).get('conscientiousness', "Person not found."), 
            neuroticism = config.PERSON_INFO_DICT.get(self.person_id, {}).get('neuroticism', "Person not found."), 
            agreeableness = config.PERSON_INFO_DICT.get(self.person_id, {}).get('agreeableness', "Person not found."), 
            extraversion = config.PERSON_INFO_DICT.get(self.person_id, {}).get('extraversion', "Person not found."), 
            history = config.PERSON_INFO_DICT.get(self.person_id, {}).get('history', "Person not found."), 
            relationships = config.PERSON_INFO_DICT.get(self.person_id, {}).get('relationships', "Person not found."), 
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
        self.pi_speech_request_subscription = self.create_subscription(
            PiSpeechRequest,
            'pi_speech_request', 
            self.pi_speech_request_callback, 
            10
        )
        self.group_info_subscription = self.create_subscription(
            GroupInfo,
            'group_info', 
            self.group_info_callback, 
            10
        )
        # Prevent unused variable warnings
        self.person_text_request_subscription 
        self.pi_speech_request_subscription
        self.group_info_subscription 

    def person_text_request_callback(self, msg):
        """
        Callback function for requesting text from the GPT for this person.
        """
        self.get_logger().info('In person_text_request_callback')
        if msg.person_id == self.person_id \
            and msg.group_id == self.group_id \
            and msg.seq > self.text_seq[msg.group_id-1]:
            text = self.person.person_speaks(
                self.person_id,
                self.group_id,
                self.group_members, # Members of the group EXCLUDING the person who will talk.
                msg.message_type
            )
            self.get_logger().info(f'Text from GPT!: {text}')
            self.person_text_result_pub(msg.seq, text)
            self.text_seq[msg.group_id-1] = msg.seq

    def person_text_result_pub(self, seq, text):
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
        for i in range(10):
            self.person_text_request_publisher.publish(msg)


    def group_info_callback(self, msg):
        """
        Callback function for receving information about group members.
        """
        self.get_logger().debug('In group_info_callback')

        if msg.seq > self.group_info_seq:
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

    def pi_speech_request_callback(self, msg):
        """
        Looks at incoming pi_speech_request data and tells the GPT about it if 
        this person is in the group (but NOT the speaker).
        """
        self.get_logger().info('In pi_speech_request_callback')
        if (msg.person_id != self.person_id) and \
            (self.person_id in msg.people_in_group) and \
            (self.group_id == msg.group_id) and \
            (msg.seq > self.speech_seq[msg.group_id-1]):
            self.person.other_member_text(
                msg.person_id, 
                msg.group_id, 
                msg.people_in_group, # This is everyone in the group EXCLUDING the speaker
                msg.text
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


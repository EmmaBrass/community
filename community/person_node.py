# Subscribe to faces_info topic
# Keep track of what system state we are in
# Publish state to system_state topic

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from community_msgs.msg import (
    PiSpeechRequest,
    PersonTextRequest,
    GroupInfo
)
import constants
import cv2, math, time, logging, pickle
import numpy as np

from person import Person


class PersonNode(Node):

    def __init__(
        self, 
        person_id: int,
        name: str, 
        age: int, 
        openness: int, 
        conscientiousness: int, 
        neuroticism: int, 
        agreeableness: int, 
        extraversion: int,
        history: str,
        interactions: dict
    ):
        super().__init__('person_node')

        self.logger = logging.getLogger("main_logger")

        self.person = Person(
            person_id,
            name, 
            age, 
            openness, 
            conscientiousness, 
            neuroticism, 
            agreeableness, 
            extraversion, 
            history, 
            interactions
        )

        self.person_id = person_id
        self.group_id = None # will change
        self.pi_id = None # will change
        self.group_members = [] # People in the group EXCLUDING this person
        # Seq list for receiving text requests - one item for each group
        self.text_seq = [-1]*constants.NUM_GROUPS #TODO check syntactically correct...
        # Seq for receiving group info
        self.group_info_seq = -1
        # Seq list for receiving speech updates - one item for each group
        self.speech_seq = [-1]*constants.NUM_GROUPS #TODO check syntactically correct...
        
        # Initialise publishers
        self.pi_speech_request_publisher = self.create_publisher(
            PiSpeechRequest, 
            'pi_speech_request', 
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
        self.logger().info('In person_text_request_callback')
        if msg.person_id == self.person_id \
            and msg.group_id == self.group_id \
            and msg.seq > self.text_seq[msg.group_id]:
            text = self.person.person_speaks(
                self.person_id,
                self.group_id,
                self.group_members # Members of the group EXCLUDING this person.
            )
            self.pi_speech_request_pub(msg.seq, text)
            self.text_seq[msg.group_id] = msg.seq

    def pi_speech_request_pub(self, seq, text):
        """
        Publish text to the pi_speech_request topic.

        :param text: The text that needs to be spoken.
        """
        self.logger().info('In pi_speech_request_pub')
        for i in range(5):
            self.pi_speech_request_publisher.publish(
                seq = seq, 
                voice_id = self.get_voice_id(self.person_id),
                person_id = self.person_id,
                pi_id = self.pi_id,
                group_id = self.group_id,
                people_in_group = self.group_members,
                text = text
            )

    def group_info_callback(self, msg):
        """
        Callback function for receving information about group members.
        """
        self.logger().info('In group_info_callback')

        if msg.seq > self.group_info_seq:
            if self.person_id in msg.person_ids:
                # Find the others in the group from the msg
                others_in_group = [person for person in msg.person_ids if person != self.person_id]
                # Get the assigned pi for this person
                assigned_pi = msg.pi_ids[msg.person_ids.index(self.person_id)]
                # Check if the person has been moved to a different pi
                if assigned_pi != self.pi_id:
                    # They have been moved
                    # Update the pi id
                    self.pi_id = assigned_pi
                # If they are in the same group as before
                if msg.group_id == self.group_id:
                    # Check if a person left the group, to send to the gpt as metadata
                    for person in self.group_members:
                        if person not in others_in_group:
                            self.group_members.remove(person)
                            # Tell the GPT about the member who left the group
                            self.person.member_left_group(person)
                    # Check for new person added, to send to the gpt as metadata
                    for person in others_in_group:
                        if person not in self.group_members: 
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
        this person is in the group (but NOT the speaker)
        """
        self.logger().info('In pi_speech_request_callback')
        if (msg.person_id != self.person_id) and \
            (self.person_id in msg.people_in_group) and \
            (self.group_id == msg.group_id) and \
            (msg.seq > self.speech_seq[msg.group_id]):
            self.person.other_member_text(
                msg.person_id, 
                msg.group_id, 
                msg.people_in_group, # This is everyone in the group EXCLUDING the speaker
                msg.text
            )
            self.speech_seq[msg.group_id] = msg.seq

    def get_voice_id(self, person_id):
        voice_id = None
        for person in constants.PERSON_ID_NAME_VOICE:
            if person['person_id'] == person_id:
                voice_id = person['voice_id']
                break
        if voice_id == None:
            print("Error! Voice_id not found for this person_id")
        return voice_id


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


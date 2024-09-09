
# Proxy group node
# 
# Subscribe to pi_person_updates topic
# Subscribe to pi_speech_complete topic
# 
# Publish to person_text_request topic

# The group node will orchestrate requests for the people in a group to speech.
# Publish the request and then only send through the next request once the previous one is complete.
# Each text request will have an ID, consisting of the group number and a message ID
# E.g. 5_12 (group 5, message 12)
# This should be included in the message sent from the pi to the pi_speech_complete topic to update when that message is spoken


# Subscribe to faces_info topic
# Keep track of what system state we are in
# Publish state to system_state topic

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from community_interfaces.msg import (
    PiSpeechComplete,
    PersonTextRequest,
    GroupInfo
)
import community.configuration as config

import cv2, math, time, logging, pickle, random
import numpy as np


class GroupNode(Node):

    def __init__(self):

        super().__init__('group_node')

        # Get group_id from launch file
        self.declare_parameter('group_id', 0)
        self.group_id = self.get_parameter('group_id').get_parameter_value().integer_value

        # The max number of people in this group
        # TODO delete? num_members not used currently
        self.num_members = len(config.GROUP_PI_ASSIGNMENTS.get(self.group_id).get('pi_ids'))
        # The people in this group (this changes)
        self.group_members = []
        # Any new members added
        self.new_members = []
        # Any members left
        self.left_members = []
        # Variable for if last requested text is complete
        self.last_text_completed = True
        # Seq for sending text requests
        self.text_seq = 0
        # Seq for receiving group info
        self.group_info_seq = -1

        # Initialise publisher
        self.person_text_request_publisher = self.create_publisher(PersonTextRequest, 'person_text_request', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback

        # Initialise subscribers
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
        self.get_logger().debug('In group_info_callback')
        if msg.seq > self.group_info_seq:
            if msg.group_id == self.group_id:
                self.get_logger().info('In Here')
                # Check for people who have left, if there is >0 people in the group
                if len(self.group_members) != 0:
                    for person_id in self.group_members:
                        if person_id not in msg.person_ids:
                            self.group_members.remove(person_id)
                            self.left_members.append(person_id)
                # Check for new person added, if there are any other members of the group
                self.get_logger().info(f'np.count_nonzero(msg.person_ids) {np.count_nonzero(msg.person_ids)}')
                if np.count_nonzero(msg.person_ids) != 0:
                    self.get_logger().info('In Here2')
                    for person_id in msg.person_ids:
                        if person_id not in self.group_members and person_id != 0:
                            self.get_logger().info('In Here3')
                            self.group_members.append(person_id)
                            self.new_members.append(person_id)
                            self.get_logger().info(f'self.new_members {self.new_members}')
            self.group_info_seq = msg.seq

    def pi_speech_complete_callback(self, msg):
        """
        Callback for info that the pi has finished speaking a requested text.
        """
        self.get_logger().info('In pi_speech_complete_callback')
        if msg.seq == self.text_seq and msg.group_id == self.group_id:
            if msg.complete == True:
                self.last_text_completed = True
                self.text_seq += 1

    def timer_callback(self):
        """
        Every timer_period seconds, check if a next text request is needed.
        If yes, choose the next person to request text from and request it.
        """
        # Check if new text required (if last person's text has been spoken).
        if self.last_text_completed == True:
            self.get_logger().info('In Here6')
            # TODO add in some time check so we are not saying hello/goodbye too often...
            # if people are leaving and coming very quickly then skip the introductions and goodbyes
            # and just ask for straight-forward conversation.

            # Check if any people who have left: only say 
            # goodbye if they are not ALSO in the new_members list, otherwise 
            # they may have been added and removed too quickly for even a hello/goodbye
            if len(self.left_members) != 0:
                # Person leaving says goodbye / feeling sleepy
                person_id = self.left_members.pop(0)
                if person_id not in self.new_members:
                    msg = PersonTextRequest()
                    msg.seq = self.text_seq
                    msg.person_id = person_id
                    msg.group_id = self.group_id
                    msg.message_type = 1
                    for i in range(10):
                        self.person_text_request_publisher.publish(msg)
                    self.last_text_completed = False
            # Check if any people who have joined
            # Check they also didn't just leave (too soon to say hello again)
            elif len(self.new_members) != 0:
                self.get_logger().info('In Here4')
                # Person joining says hello
                person_id = self.new_members.pop(0)
                if person_id not in self.left_members:
                    self.get_logger().info('In Here5')
                    msg = PersonTextRequest()
                    msg.seq = self.text_seq
                    msg.person_id = person_id
                    msg.group_id = self.group_id
                    msg.message_type = 0
                    for i in range(10):
                        self.person_text_request_publisher.publish(msg)
                    self.last_text_completed = False
            # Otherwise, choose a person at random to request text from
            else:
                if len(self.group_members) > 0: # If not the very beginning of the run when noone has been put in the group yet.
                    person_id = random.choice(self.group_members)# TODO more sophisticated picking technique
                    msg = PersonTextRequest()
                    msg.seq = self.text_seq
                    msg.person_id = person_id
                    msg.group_id = self.group_id
                    msg.message_type = 2
                    for i in range(10):
                        self.person_text_request_publisher.publish(msg)
                    self.last_text_completed = False
            
            

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



    


    
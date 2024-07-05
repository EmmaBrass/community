
# A node that looks at pi_person_updates topic
# Then updates a database that says who is in what list.
# Then continuously publishes this updated list.
# TODO what format to publish list in... maybe each person ID and then the ID of what pi they are at
# Then we have a SAVED list of what pi is in what group and can use this for assigning each person to each group


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

from community_msgs.msg import (
    PiPersonUpdates,
    GroupInfo
)

import cv2, math, time, logging, pickle, random
import numpy as np
import constants


class GroupAssignmentNode(Node):

    def __init__(self, group_id: int, num_members: int):
        super().__init__('group_assignment_node')

        self.logger = logging.getLogger("main_logger")

        self.group_info_seq = 0

        # Initialize the assignments list
        self.pi_person_assignment = []
        # Loop through each group in the original list
        for group in constants.GROUP_PI_ASSIGNMENTS:
            group_id = group['group_id']
            pi_ids = group['pi_ids']
            # Create the members list for the current group
            members = [{'pi_id': pi_id, 'person_id': -1} for pi_id in pi_ids]
            # Append the new group structure to the result list
            self.pi_person_assignment.append({'group_id': group_id, 'members': members})

        # Initialise publisher
        self.group_info_publisher = self.create_publisher(GroupInfo, 'group_info', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback

        # Initialise subscribers
        self.pi_person_updates_subscription = self.create_subscription(
            PiPersonUpdates,
            'pi_person_updates', 
            self.pi_person_updates_callback, 
            10
        )
        # Prevent unused variable warnings
        self.pi_person_updates_subscription

    def update_pi_person_assignment(self, pi_id: int, new_person_id:int):
        # Loop through each group in the assignments
        for group in self.pi_person_assignment:
            # Loop through each member in the group
            for member in group['members']:
                # Check if the current member has the specified pi_id
                if member['pi_id'] == pi_id:
                    # Update the person_id
                    member['person_id'] = new_person_id
                    return True  # Return True if the update was successful
        return False  # Return False if the pi_id was not found

    def pi_person_updates_callback(self, msg):
        """
        Callback for info on what person is assigned to what pi.
        """
        # Update the pi/person assignments list
        success = self.update_pi_person_assignment(msg.pi_id, msg.person_id)
        if success == False:
            self.logger.warning("Pi ID not found!")

    def timer_callback(self):
        """
        Every timer_period seconds, submit messages to group_info topic.
        """
        for group in self.pi_person_assignment:
            for i in range(5):
                self.group_info_publisher.publish(
                    seq = self.group_info_seq,
                    group_id = group['group_id'], 
                    person_ids = [member['person_id'] for member in group['members']], 
                    pi_ids = [member['pi_id'] for member in group['members']]
                )
            self.group_info_seq += 1

def main(args=None):
    rclpy.init(args=args)

    group_assignment_node = GroupAssignmentNode()

    rclpy.spin(group_assignment_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    group_assignment_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



    


    

# A node that looks at pi_person_updates topic
# Then updates a database that says who is in what list.
# Then continuously publishes this updated list.


import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from community_interfaces.msg import (
    PiPersonUpdates,
    GroupInfo
)
import community.config_files.configuration as config

import cv2, math, time, logging, pickle, random
import numpy as np


class GroupAssignmentNode(Node):

    def __init__(self):
        super().__init__('group_assignment_node')

        self.group_info_seq = 0

        # Initialize the assignments list for what person is at what pi
        # Very similar to GROUP_PI_ASSIGNMENTS but also gives a person_id for each pi_id
        # This person_id will be updated as the RFID cards are moved around
        # Initially all pis will be assigned person_id of -1 (no person)
        self.pi_person_assignments = []
        # Loop through each group in the original list
        for group_id, group_info in config.GROUP_PI_ASSIGNMENTS.items():
            pi_ids = group_info.get('pi_ids', [])
            members = [{'pi_id': pi_id, 'person_id': 0} for pi_id in pi_ids]
            self.pi_person_assignments.append({'group_id': group_id, 'members': members})

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

    def update_pi_person_assignments(self, pi_id: int, new_person_id:int):
        """
        Update the person_id assigned to a specific pi_id in self.pi_person_assignments.

        :param pi_id: The pi_id to update.
        :param new_person_id: The new person_id to assign.
        :return: True if the update was successful, False if pi_id was not found.
        """
        # Loop through each group in the assignments
        for group in self.pi_person_assignments:
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
        success = self.update_pi_person_assignments(msg.pi_id, msg.person_id)
        if success == False:
            self.get_logger.warning("Pi ID not found!")

    def timer_callback(self):
        """
        Every timer_period seconds, submit messages to group_info topic.
        """
        # Publish for every group
        for group in self.pi_person_assignments:
            msg = GroupInfo()
            msg.seq = self.group_info_seq
            msg.group_id = group['group_id']
            msg.num_pis = len(group['members'])
            msg.person_ids = [member['person_id'] for member in group['members']]
            msg.pi_ids = [member['pi_id'] for member in group['members']]
            for i in range(5):
                self.group_info_publisher.publish(msg)
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



    


    

# A node that looks at pi_person_updates topic
# Then updates a database that says who is in what list.
# Then continuously publishes this updated list.


import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from community_interfaces.msg import (
    PiSpeechRequest,
    PiPersonUpdates,
    GroupInfo
)
import community.configuration as config
from community.message_type import MessageType
from community.helper_functions import HelperFunctions

from piper.voice import PiperVoice
import random, os, yaml, wave

from ament_index_python.packages import get_package_share_directory



class GroupAssignmentNode(Node):

    def __init__(self):
        super().__init__('group_assignment_node')

        self.restart = True # If the system has just been started up, this enables reset of seq ids on RPis.

        self.group_info_seq = 0

        self.hello_seq = 1

        # Helper functions
        self.helper = HelperFunctions()

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
        self.get_logger().info(str(self.pi_person_assignments))


        # Quick'n'easy ways to say hello when joining a group TODO more customisation of this somehow?
        self.hello_list = ['Hello there! What a nice day it is.', 
                           'Hellooooo good people of the world.', 
                           'Hey, glad to be here with you.', 
                           'Hey, it\'s nice to be here, I wouldn\'t want to be anywhere else.', 
                           'Hello there. It\'s great to be here with you!', 
                           'Hi, looking forward to talking to you about interesting things.',
                           'Howdy folks, I\'m so excited to have joined this group.', 
                           'I\'m so happy to be here in this group, I cannot wait.'
                           ]

        # Initialise publisher
        self.group_info_publisher = self.create_publisher(GroupInfo, 'group_info', 10)
        self.pi_speech_request_publisher = self.create_publisher(PiSpeechRequest, 'pi_speech_request', 10)
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
            current_group_id = group['group_id']
            # Loop through each member in the group
            for member in group['members']:
                # Check if the current member has the specified pi_id
                if member['pi_id'] == pi_id:
                    if member['person_id'] != new_person_id:
                        member['person_id'] = new_person_id
                        self.get_logger().info("new_person_id")
                        self.get_logger().info(str(new_person_id))
                        if new_person_id != 0:
                            voice_id = self.helper.get_voice_id(new_person_id)
                            color = self.helper.get_color(new_person_id)
                            # Convert text to .wav audio file bytes.
                            text = random.choice(self.hello_list)
                            audio_uint8 = self.helper.text_to_speech_bytes(text, voice_id, "hello")
                            # Publish a PiSpeechRequest so that the person can say hello
                            self.pi_speech_request_pub(new_person_id, pi_id, color, current_group_id, voice_id, text, audio_uint8)
                    return True  # Return True if an update took place
        return False  # Return False if the pi_id was not found in any group
    
    def pi_speech_request_pub(self, person_id, pi_id, color, group_id, voice_id, text, audio_uint8):
        msg = PiSpeechRequest()
        msg.seq = self.hello_seq
        msg.voice_id = voice_id
        msg.person_id = person_id
        msg.pi_id = pi_id
        msg.color = color
        msg.group_id = group_id
        msg.people_in_group = []
        msg.message_type = MessageType.HELLO.value
        msg.text = text
        msg.gpt_message_id = "0"
        msg.directed_id = 0
        msg.relationship_ticked = False
        msg.relationship_tick = 0
        msg.chaos_phase = False
        msg.audio_data = audio_uint8
        for i in range(5):
            self.pi_speech_request_publisher.publish(msg)
        self.hello_seq += 1

    def pi_person_updates_callback(self, msg):
        """
        Callback for info on what person is assigned to what pi.
        """
        # Update the pi/person assignments list
        success = self.update_pi_person_assignments(msg.pi_id, msg.person_id)
        if success == False:
            self.get_logger().error("Pi ID not found in any group!")

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
            if self.restart == True: # If the system has just been started up, this enables reset of seq ids on RPis.
                msg.restart = True
                self.restart = False
            else:
                msg.restart = False
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



    


    

# A node that looks at pi_person_updates topic
# Then updates a database that says who is in what list.
# Then continuously publishes this updated list.


import rclpy
from rclpy.node import Node
from rclpy.task import Future

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from community_interfaces.msg import (
    PiPersonUpdates,
    GroupInfo
)
from community_interfaces.srv import (
    PiSpeechRequest
)

import community.configuration as config
from community.message_type import MessageType
from community.helper_functions import HelperFunctions
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from piper.voice import PiperVoice
import random, os, yaml, wave

from community.configuration import PEOPLE_TO_USE

from ament_index_python.packages import get_package_share_directory


class GroupAssignmentNode(Node):

    def __init__(self):
        super().__init__('group_assignment_node')

        self.restart = True # If the system has just been started up, this enables reset of seq ids on RPis.

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

        # Quick'n'easy ways to say hello when joining a group TODO more customisation of this somehow? -> with their own name.
        self.hello_list = ['Hello there! What a nice day it is. I am ', 
                           'Hello good people of the world. My name is ', 
                           'Hey, glad to be here with you. My name is ', 
                           'Hey, it is nice to be here, I am ', 
                           'Hello, it is great to be here with you! I am ', 
                           'Hi, looking forward to talking to you about interesting things. I am ',
                           'Howdy folks, I am so excited to have joined this group. I am ', 
                           'I am so happy to be here in this group, I cannot wait. My name is '
                           ]
        
        # Create callback groups
        callback_group_1 = MutuallyExclusiveCallbackGroup()
        callback_group_2 = MutuallyExclusiveCallbackGroup()
        callback_group_3 = MutuallyExclusiveCallbackGroup()
        callback_group_4 = MutuallyExclusiveCallbackGroup()

        # Initialise timer callback
        # Publishing happens within the timer_callback
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(
            timer_period, 
            self.timer_callback,
            callback_group=callback_group_1
        ) 
        # Initialise timer callback for checking the status of the pi services
        pi_timer_period = 0.5  # seconds
        self.pi_timer = self.create_timer(
            pi_timer_period, 
            self.pi_timer_callback,
            callback_group=callback_group_2
        ) 
        # Initialise publisher
        self.group_info_publisher = self.create_publisher(
            GroupInfo, 
            'group_info', 
            10,
            callback_group=callback_group_3
        )
        # Initialise subscriber
        self.pi_person_updates_subscription = self.create_subscription(
            PiPersonUpdates,
            'pi_person_updates', 
            self.pi_person_updates_callback, 
            10,
            callback_group=callback_group_4
        )

        # Initialise service clients
        # Extract all pi_ids
        all_pi_ids = [pi_id for group in config.GROUP_PI_ASSIGNMENTS.values() for pi_id in group['pi_ids']]
        # Dictionaries to store service clients, and callback groups
        self.pi_speech_request_clients = {}
        self.pi_service_status = {pi_id: False for pi_id in all_pi_ids}
        self.pi_speech_request_callback_groups = {}
        # Create a service client and callback group for each pi in this group
        for pi_id in all_pi_ids:
            # Unique service name per Pi
            service_name = f'pi_speech_request_{pi_id}'  
            # Create a separate MutuallyExclusiveCallbackGroup for each client
            self.pi_speech_request_callback_groups[pi_id] = MutuallyExclusiveCallbackGroup()
            # Create a service client bound to this callback group
            client = self.create_client(
                PiSpeechRequest, 
                service_name, 
                callback_group=self.pi_speech_request_callback_groups[pi_id]
            )
            # Store references
            self.pi_speech_request_clients[pi_id] = client
            # Check if the client is available, but only breifly
            ready = client.wait_for_service(timeout_sec=1.0)
            if ready:
                self.pi_service_status[pi_id] = True
                self.get_logger().info(f'Service {service_name} is available!')
            else:
                self.pi_service_status[pi_id] = False
                self.get_logger().warning(f'Service {service_name} is offline!')

    def check_pi_services(self):
        """
        Check if the Pi services are available and update status.
        Reset the assigned person_id to 0 if the pi goes offline or comes online.
        """
        for pi_id, client in self.pi_speech_request_clients.items():
            if client.service_is_ready():
                if not self.pi_service_status[pi_id]:
                    self.get_logger().info(f'Service pi_speech_request_{pi_id} is now available!')
                    # Reset the assigned person_id to 0 for this pi
                    for group in self.pi_person_assignments:
                        for member in group['members']:
                            # Check if the current member has the specified pi_id
                            if member['pi_id'] == pi_id:
                                member['person_id'] = 0
                self.pi_service_status[pi_id] = True
            else:
                if self.pi_service_status[pi_id]:
                    self.get_logger().warning(f'Service pi_speech_request_{pi_id} went offline!')
                    # Reset the assigned person_id to 0 for this pi
                    for group in self.pi_person_assignments:
                        for member in group['members']:
                            # Check if the current member has the specified pi_id
                            if member['pi_id'] == pi_id:
                                member['person_id'] = 0
                self.pi_service_status[pi_id] = False

    def pi_person_updates_callback(self, msg):
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
                if member['pi_id'] == msg.pi_id:
                    if member['person_id'] != msg.person_id: # TODO needs to still work if pi restarted!
                        member['person_id'] = msg.person_id
                        self.get_logger().info("new_person_id")
                        self.get_logger().info(str(msg.person_id))
                        if msg.person_id != 0:
                            if msg.person_id not in PEOPLE_TO_USE:
                                self.get_logger().error(f"RFID is not in list of people to use! {msg.person_id}")
                                return False
                            voice_id = self.helper.get_voice_id(msg.person_id)
                            color = self.helper.get_color(msg.person_id)
                            name = self.helper.get_name(msg.person_id)
                            # Convert text to .wav audio file bytes.
                            text = random.choice(self.hello_list) + str(name)
                            self.get_logger().info(f"Text-to-speech input: text={text}, voice_id={voice_id}")
                            audio_uint8 = self.helper.text_to_speech_bytes(text, voice_id, "hello", volume=1.0)
                            if not all(isinstance(b, int) and 0 <= b <= 255 for b in audio_uint8):
                                self.get_logger().error(f"Invalid audio data: {audio_uint8}")
                            # Use PiSpeechRequest client so that the person can say hello
                            if self.pi_service_status.get(msg.pi_id, False): # Check that the pi is online before sending 'hello' speech
                                self.pi_speech_request(msg.person_id, msg.pi_id, color, current_group_id, voice_id, text, audio_uint8)
                    return True  # Return True if an update took place
        self.get_logger().error("Pi ID not found in any group!")
        return False  # Return False if the pi_id was not found in any group
    
    def pi_speech_request(self, person_id, pi_id, color, group_id, voice_id, text, audio_uint8):
        request = PiSpeechRequest.Request()
        request.person_id = person_id
        request.pi_id = pi_id
        request.color = color
        request.group_id = group_id
        request.voice_id = voice_id
        request.text = text
        request.audio_data = audio_uint8
        future = self.pi_speech_request_clients[pi_id].call_async(request)
        future.add_done_callback(self.pi_speech_request_callback)

    def pi_speech_request_callback(self, future: Future):
        """
        Callback for the future, that will be called when the request is done.
        """
        response = future.result()
        if response.completed == True:
            self.get_logger().info("Pi speech request completed.")
        else:
            self.get_logger().info("Pi speech request not completed.")

    def pi_timer_callback(self):
        """
        Check and update the current status of the pi services.
        """
        self.check_pi_services()

    def timer_callback(self):
        """
        Every timer_period seconds, submit messages to group_info topic.
        """
        # Publish for every group
        for group in self.pi_person_assignments:
            msg = GroupInfo()
            msg.group_id = group['group_id']
            msg.num_pis = len(group['members'])
            msg.person_ids = [member['person_id'] for member in group['members']]
            msg.pi_ids = [member['pi_id'] for member in group['members']]
            if self.restart == True: # If the system has just been started up, this enables reset of seq ids on RPis.
                msg.restart = True
                self.restart = False
            else:
                msg.restart = False
            self.group_info_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    group_assignment_node = GroupAssignmentNode()
    # Use MultiThreadedExecutor to allow parallel execution
    executor = MultiThreadedExecutor()

    rclpy.spin(group_assignment_node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    group_assignment_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



    


    
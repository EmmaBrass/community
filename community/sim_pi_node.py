
### Simulated pi node for running simulation of one group on computer ###
# Keyboard inputs for adding and removing people to pis #

# Up to 9 heads in the group #
# Press a number to access that head #
# If there is already a card there, pressing the number will remove the card #
# If there is no card there, then asked to select a person to add from a dropdown menu #

import rclpy
from rclpy.node import Node
from rclpy.task import Future 

import subprocess, time

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from community_interfaces.msg import (
    PiPersonUpdates,
    SimPiPersonAssign
)
from community_interfaces.srv import (
    PiSpeechRequest
)

# Message types:
# HELLO = 0 # Say hello because just joined a group
# GOODBYE = 1 # Say bye because leaving a group (likely won't be used...)
# OTHERS = >=2

class SimPiNode(Node):

    def __init__(self):
        super().__init__('pi_node')

        # Get pi_id from launch file
        self.declare_parameter('pi_id', 0)
        self.pi_id = self.get_parameter('pi_id').get_parameter_value().integer_value

        # The ID for the RFID object places on the pi (this changes).
        self.person_id = 0 # 0 means no card there

        # For tracking audio playback
        self.playback_process = None

        # Flag indicating if audio playback is finished
        self.audio_finished = True  

        # Initialise publishers
        self.pi_person_updates_publisher = self.create_publisher(PiPersonUpdates, 'pi_person_updates', 10)

        # Use mutually exclusive callback group to avoid overlapping callbacks
        callback_group_1 = MutuallyExclusiveCallbackGroup()
        callback_group_2 = MutuallyExclusiveCallbackGroup()
        callback_group_3 = MutuallyExclusiveCallbackGroup()
        self.callback_group_4 = MutuallyExclusiveCallbackGroup()
        callback_group_5 = MutuallyExclusiveCallbackGroup()

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group_1) # Publishing happens within the timer_callback

        # Add a timer to monitor playback
        self.playback_timer = self.create_timer(0.1, self.monitor_playback, callback_group_2)

        # Initialise service servers
        self.pi_speech_request = self.create_service(
            PiSpeechRequest, 
            f'pi_speech_request_{self.pi_id}', 
            self.pi_speech_request_callback,
            callback_group=callback_group_3
        )

        # Initialise subscribers
        self.sim_pi_person_assign_subscription = self.create_subscription(
            SimPiPersonAssign,
            'sim_pi_person_assign', 
            self.sim_pi_person_assign_callback, 
            10,
            callback_group=callback_group_5
        )

    def data_to_speech(self, audio_data):
        """
        Convert audio data into a .wab file and then play the .wav file.
        """
        file_path = 'output.wav'
        try:
            audio_bytes = bytes(audio_data)
            with open(file_path, 'wb') as f:
                f.write(audio_bytes)
            self.get_logger().info("Saved output .wav file.")
        except Exception as e:
            self.get_logger().error(f"Failed to save audio file: {e}")
            self.audio_finished = True

        # Play audio
        self.get_logger().info("Playing .wav file")
        self.playback_process = subprocess.Popen(['aplay', file_path])

    def stop_audio_playback(self):
        """
        Forced stop audio playback if a process is running.
        """
        if self.playback_process and self.playback_process.poll() is None:  # Process is still running
            self.playback_process.terminate()
            self.get_logger().info("Stopped playback.")
            self.playback_process = None
        self.audio_finished = True

    def monitor_playback(self):
        """
        Monitor the playback process and update the finished flag when playback ends naturally.
        """
        if self.playback_process and self.playback_process.poll() is not None:  # Playback finished naturally
            self.get_logger().info("Playback completed.")
            self.audio_finished = True
            self.playback_process = None

    def sim_pi_person_assign_callback(self, msg):
        """
        Update what person is on this simulated pi node.
        """
        if msg.pi_id == self.pi_id and msg.person_id != self.person_id:
            # Stop any playback if the person ID changes
            self.stop_audio_playback()
            # Update person_id
            self.person_id = msg.person_id

    def pi_speech_request_callback(self, request, response):
        """
        Callback function for a speech request for the pi speakers.
        When complete, calls function to send 'complete' message.
        """
        self.get_logger().info('In pi_speech_request_callback')

        # Check if the message is for THIS pi.
        # If yes, submit the text to the speakers.
        if self.person_id == request.person_id: # Current person matches the request
            self.get_logger().info(f'Requesting audio play for .wav data: {request.text}')
            self.audio_finished = False
            self.data_to_speech(request.audio_data)

            while not self.audio_finished:
                time.sleep(0.5)

            response.completed = True
            return response

        else:
            self.get_logger().info('Person for whom speech was requested is no longer on this Pi.')
            response.completed = False
            return response

        # # Set the result in the future to send the response
        # future.set_result(response)

        # if self.audio_finished:  # If audio has finished
        #     self.get_logger().info("Audio processing finished. Sending response.")

        #     # Destroy the timer
        #     if self.audio_check_timer is not None:
        #         self.audio_check_timer.cancel()
        #         self.destroy_timer(self.audio_check_timer)
        #         self.audio_check_timer = None  # Reset timer reference

        #     # Send response
        #     if self.pending_response is not None:
        #         self.pending_response.completed = True

        #         # Set the Future result to complete the service response
        #         self.pending_future.set_result(self.response)

        #         # Clear stored response
        #         self.pending_future = None

    def timer_callback(self):
        """
        Every x seconds, publish the ID for the RFID object on this pi at the moment.
        (Or the absense of any RFID number: 0).
        """
        # Publish assigned person_id.
        self.get_logger().debug("Publishing current assigned person_id for this pi.")
        # Check if a card is available to read
        msg = PiPersonUpdates()
        msg.pi_id = self.pi_id
        msg.person_id = self.person_id
        for i in range(5):
            self.pi_person_updates_publisher.publish(msg)

    
def main(args=None):
    rclpy.init(args=args)

    sim_pi_node = SimPiNode()
    # Use MultiThreadedExecutor to allow parallel execution
    executor = MultiThreadedExecutor()

    rclpy.spin(sim_pi_node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim_pi_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



    


    

### Simulated pi node for running simulation of one group on computer ###
# Keyboard inputs for adding and removing people to pis #

# Up to 9 heads in the group #
# Press a number to access that head #
# If there is already a card there, pressing the number will remove the card #
# If there is no card there, then asked to select a person to add from a dropdown menu #

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import wave, os, random, subprocess
from piper.voice import PiperVoice

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from community_interfaces.msg import (
    PiPersonUpdates,
    PiSpeechComplete,
    PiSpeechRequest,
    GroupInfo,
    SimPiPersonAssign
)


class SimPiNode(Node):

    def __init__(self):
        super().__init__('pi_node')

        # Get pi_id from launch file
        self.declare_parameter('pi_id', 0)
        self.pi_id = self.get_parameter('pi_id').get_parameter_value().integer_value

        # The ID for the RFID object places on the pi (this changes).
        self.person_id = 0 # 0 means no card there

        self.playback_process = None
        self.stop_playback = False

        # Use mutually exclusive callback group to avoid overlapping callbacks
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Seq list for receiving speech requests - one item for each group
        self.speech_seq = None

        # Seq for chaos phase
        self.chaos_seq = 0

        # Flag indicating if audio playback is finished
        self.audio_finished = True  

        # Initialise publishers
        self.pi_person_updates_publisher = self.create_publisher(PiPersonUpdates, 'pi_person_updates', 10)
        self.pi_speech_complete_publisher = self.create_publisher(PiSpeechComplete, 'pi_speech_complete', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback

        # Add a timer to monitor playback
        self.playback_timer = self.create_timer(0.1, self.monitor_playback, callback_group=self.callback_group)

        # Quick'n'easy ways to say hello when joining a group TODO more customisation of this somehow?
        self.hello_list = ['Hi there!', 
                           'Hello', 
                           'Hey guys', 
                           'Hey, it\'s nice to be here', 
                           'Hello there. It\'s great to be here!', 
                           'Hi, looking forward to talking to you all',
                           'Howdy folks', 
                           'I\'m so happy to be here'
                           ]

        # Initialise subscribers
        self.sim_pi_person_assign_subscription = self.create_subscription(
            SimPiPersonAssign,
            'sim_pi_person_assign', 
            self.sim_pi_person_assign_callback, 
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
        self.pi_speech_request_subscription 


    def text_to_speech(self, to_speak, voice_id):
        """
        Speak a text file
        """
        try:
            voicedir = os.path.expanduser('~/Documents/piper/')  # Model directory
            model = voicedir + voice_id
            voice = PiperVoice.load(model)
            
            # Define output .wav file
            wav_file = 'output.wav'
            
            # Open wave file in binary write mode
            with wave.open(wav_file, 'wb') as wav:
                # Set wave parameters (sample width, channels, frame rate)
                wav.setnchannels(1)  # Mono audio
                wav.setsampwidth(2)  # Typically 16-bit audio
                wav.setframerate(22050)  # Set frame rate to 22.05 kHz
                
                # Synthesize text and write audio frames
                voice.synthesize(to_speak, wav)

            # Play audio
            self.get_logger().info("Playing .wav file")
            self.playback_process = subprocess.Popen(['aplay', wav_file])
            self.stop_playback = False

        except Exception as e:
            self.get_logger().error(f"Error in text_to_speech: {e}")
            self.audio_finished = True  # Set to finished in case of error

    def stop_audio_playback(self):
        """
        Stop audio playback if a process is running.
        """
        if self.playback_process and self.playback_process.poll() is None:  # Process is still running
            self.playback_process.terminate()
            self.get_logger().info("Stopped playback.")
            self.playback_process = None
        self.stop_playback = True
        self.audio_finished = True

    def monitor_playback(self):
        """
        Monitor the playback process and update the finished flag when playback ends.
        """
        if self.playback_process and self.playback_process.poll() is not None:  # Playback finished naturally
            self.get_logger().info("Playback completed.")
            self.audio_finished = True
            self.playback_process = None
        elif self.stop_playback:
            self.stop_audio_playback()  # Handle forced stop

    def sim_pi_person_assign_callback(self, msg):
        """
        Update what person is on this simulated pi node.
        """
        if msg.pi_id == self.pi_id and msg.person_id != self.person_id:
            # Stop any playback if the person ID changes
            self.stop_audio_playback()
            # Update person_id
            self.person_id = msg.person_id
            if self.person_id != 0:  # Say hello if a new person is assigned
                self.text_to_speech(random.choice(self.hello_list), msg.voice_id)

    def group_info_callback(self, msg):
        """
        Used just after initialisation to set the number of pis
        in this group (one speech_seq instance for each)
        """
        if self.speech_seq == None:
            self.get_logger().info('Setting speech_seq list')
            # Count the number of members of the group and set the speech_seq
            self.speech_seq = [-1]*msg.num_pis # Seq list for receiving speech requests - one item for each group

    def pi_speech_request_callback(self, msg):
        """
        Callback function for a speech request for the pi speakers.
        When complete, calls function to send 'complete' message.
        """
        self.get_logger().info('In pi_speech_request_callback')

        if self.speech_seq is not None:
            # Check if the message is for THIS pi.
            # If yes, submit the text to the speakers.
            if msg.pi_id == self.pi_id and (
                (msg.chaos_phase == False and msg.seq > self.speech_seq[msg.group_id]) or
                (msg.chaos_phase == True and msg.seq > self.chaos_seq)
            ):
                if self.person_id == msg.person_id: # Current person matches the request
                    self.get_logger().info(f'Requesting tts for text: {msg.text}')
                    self.audio_finished = False
                    self.text_to_speech(msg.text, msg.voice_id)
                    # Start a timer to check for audio completion
                    self.audio_check_timer = self.create_timer(
                        0.1,  # Check every 0.1 seconds
                        lambda: self.check_audio_and_complete(
                            msg.seq,
                            msg.person_id,
                            msg.pi_id,
                            msg.group_id,
                            msg.people_in_group,
                            msg.text,
                            msg.gpt_message_id,
                            msg.directed_id,
                            msg.relationship_ticked,
                            msg.relationship_tick,
                            msg.mention_question
                        )
                    )

                else:
                    self.get_logger().info('Person for whom speech was requested is no longer on this Pi.')
                    self.pi_speech_complete(False, msg.seq, msg.person_id, msg.pi_id, msg.group_id,
                                            msg.people_in_group, msg.text, msg.gpt_message_id,
                                            msg.directed_id, msg.relationship_ticked,
                                            msg.relationship_tick, msg.mention_question)
                if not msg.chaos_phase:
                    self.speech_seq[msg.group_id] = msg.seq
                else:
                    self.chaos_seq = msg.seq

    def check_audio_and_complete(self, seq, person_id, pi_id, group_id, people_in_group, text, gpt_message_id, directed_id, relationship_ticked, relationship_tick, mention_question):
        """
        Check if the audio playback is complete and then call pi_speech_complete.
        """
        if self.audio_finished:
            self.get_logger().info("Audio playback complete. Sending pi_speech_complete message.")
            self.pi_speech_complete(
                True, seq, person_id, pi_id, group_id, people_in_group, text, gpt_message_id,
                directed_id, relationship_ticked, relationship_tick, mention_question
            )
            # Stop the timer after completion
            if hasattr(self, 'audio_check_timer'):
                self.destroy_timer(self.audio_check_timer)
                del self.audio_check_timer
                
    def pi_speech_complete(self, complete, seq, person_id, pi_id, group_id, people_in_group, text, gpt_message_id, directed_id, relationship_ticked, relationship_tick, mention_question):
        """
        Publish to say that the text has been spoken.
        """
        msg = PiSpeechComplete()
        msg.seq = seq
        msg.person_id = person_id
        msg.pi_id = pi_id
        msg.group_id = group_id
        msg.people_in_group = people_in_group
        msg.text = text
        msg.gpt_message_id = gpt_message_id
        msg.directed_id = directed_id
        msg.complete = complete
        msg.relationship_ticked = relationship_ticked
        msg.relationship_tick = relationship_tick
        msg.mention_question = mention_question
        for i in range(5):
            self.pi_speech_complete_publisher.publish(msg)

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

    rclpy.spin(sim_pi_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim_pi_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



    


    
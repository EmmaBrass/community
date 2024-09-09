
# Subscribe to faces_info topic
# Keep track of what system state we are in
# Publish state to system_state topic

import board
import busio
from digitalio import DigitalInOut
from adafruit_pn532.spi import PN532_SPI

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import wave, os
from piper.voice import PiperVoice

from community_interfaces.msg import (
    PiPersonUpdates,
    PiSpeechComplete
)


class PiNode(Node):

    def __init__(self):
        super().__init__('pi_node')

        # Get pi_id from launch file
        self.declare_parameter('pi_id', 0)
        self.pi_id = self.get_parameter('pi_id').get_parameter_value().integer_value

        # The ID for the RFID object places on the pi (this changes).
        self.person_id = 0 # 0 means no card there

        # Seq list for receiving speech requests - one item for each group
        self.speech_seq = None

        # SPI connection:
        self.spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
        self.cs_pin = DigitalInOut(board.D22)
        self.pn532 = PN532_SPI(self.spi, self.cs_pin, debug=False)
        ic, ver, rev, support = self.pn532.firmware_version
        self.get_logger().info("Found PN532 with firmware version: {0}.{1}".format(ver, rev))

        # Configure PN532 to communicate with MiFare cards
        self.pn532.SAM_configuration()

        # Initialise publishers
        self.pi_person_updates_publisher = self.create_publisher(PiPersonUpdates, 'pi_person_updates', 10)
        self.pi_speech_complete_publisher = self.create_publisher(PiSpeechComplete, 'pi_speech_complete', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback

        # Initialise subscribers
        self.pi_speech_request_subscription = self.create_subscription(
            String,
            'pi_speech_request', 
            self.pi_speech_request_callback, 
            10
        )
        self.group_info_subscription = self.create_subscription(
            String,
            'group_info', 
            self.group_info_callback, 
            10
        )
        # Prevent unused variable warnings
        self.pi_speech_request_subscription 

    def group_info_callback(self, msg):
        """
        Used just after initialisation to set the number of people
        in this group.
        """
        self.get_logger().info('In group_info_callback')
        if self.speech_seq == None:
            # Count the number of members of the group and set the speech_seq
            self.speech_seq = [-1]*msg.num_pis # Seq list for receiving speech requests - one item for each group

    def pi_speech_request_callback(self, msg):
        """
        Callback function for a speech request for the pi speakers.
        When complete, calls function to send 'complete' message.
        """
        self.get_logger().info('In pi_speech_request_callback')
        if self.speech_seq != None:
            # Check if the message is for THIS pi.
            # If yes, submit the text to the speakers.
            if msg.pi_id == self.pi_id and \
            (msg.seq > self.speech_seq[msg.group_id]):
                self.text_to_speech(msg.text, msg.voice_id)
                self.pi_speech_complete(msg.seq)
            self.speech_seq[msg.group_id] = msg.seq

    def pi_speech_complete(self, seq):
        """
        Publish to say that the text has been spoken.
        """
        msg = PiSpeechComplete()
        msg.seq = seq
        msg.complete = True
        for i in range(10):
            self.pi_speech_complete_publisher.publish(msg)

    def timer_callback(self):
        """
        Every x seconds, publish the ID for the RFID object on this pi at the moment.
        (Or the absense of any RFID number: 0).
        """
        # Check and update assigned RFID object.
        # TODO have a simulation version for this... for testing...
        # Publish assigned RFID object number.
        self.get_logger().info("Checking for RFID/NFC card...")
        # Check if a card is available to read
        try:
            uid = self.pn532.read_passive_target(timeout=0.5)
        except:
            self.get_logger().info("Card read error - wrong card type used?")
            self.person_id = 0
        if uid is None:
            self.get_logger().info("No card found.")
            self.person_id = 0
        else:
            uid_str = ''.join([str(i) for i in uid])
            uid_int = int(uid_str)
            self.get_logger().info(f"Found card with UID: {uid_int}")
            self.person_id = uid_int
        msg = PiPersonUpdates()
        msg.pi_id = self.pi_id
        msg.person_id = self.person_id
        for i in range(5):
            self.pi_person_updates_publisher.publish(msg)

    def text_to_speech(self, to_speak, voice_id):
        voicedir = os.path.expanduser('~/Documents/piper/') #Where onnx model files are stored on my machine
        model = voicedir+voice_id
        voice = PiperVoice.load(model)
        wav_file = wave.open('output.wav', 'w')
        text = to_speak
        audio = voice.synthesize(text,wav_file)
        # Play the .wav file
        self.get_logger().info("Playing .wav file")
        os.system('aplay output.wav')


def main(args=None):
    rclpy.init(args=args)

    pi_node = PiNode()

    rclpy.spin(pi_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pi_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



    


    
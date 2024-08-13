
# Subscribe to faces_info topic
# Keep track of what system state we are in
# Publish state to system_state topic

import board
import busio
from digitalio import DigitalInOut
from adafruit_pn532.spi import PN532_SPI

import constants

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import cv2, math, time, logging, pickle
import numpy as np
import playsound

from community_interfaces.msg import (
    PiPersonUpdates,
    PiSpeechComplete
)


class PiNode(Node):

    def __init__(self, pi_id):
        super().__init__('pi_node')

        self.logger = logging.getLogger("main_logger")

        # The id number for this pi (static) 
        self.pi_id = pi_id

        # The ID for the RFID object places on the pi (this changes).
        self.person_id = 0 # 0 means no card there

        # Seq list for receiving speech requests - one item for each group
        self.speech_seq = [-1]*constants.NUM_GROUPS

        # SPI connection:
        self.spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
        self.cs_pin = DigitalInOut(board.D5)
        self.pn532 = PN532_SPI(self.spi, self.cs_pin, debug=False)
        ic, ver, rev, support = self.pn532.firmware_version
        print("Found PN532 with firmware version: {0}.{1}".format(ver, rev))

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
        # Prevent unused variable warnings
        self.pi_speech_request_subscription 

    def pi_speech_request_callback(self, msg):
        """
        Callback function for a speech request for the pi speakers.
        When complete, calls function to send 'complete' message.
        """
        self.logger().info('In pi_speech_request_callback')
        # Check if the message is for THIS pi.
        # If yes, submit the text to the speakers.
        if msg.pi_id == self.pi_id and \
        (msg.seq > self.speech_seq[msg.group_id]):
            self.text_to_speech(msg.text)
            self.speak()
            self.pi_speech_complete(msg.seq)
        self.speech_seq[msg.group_id] = msg.seq

    def pi_speech_complete(self, seq):
        """
        Publish to say that the text has been spoken.
        """
        for i in range(10):
            self.pi_speech_complete_publisher.publish(seq=seq, complete=True)

    def timer_callback(self):
        """
        Every x seconds, publish the ID for the RFID object on this pi at the moment.
        (Or the absense of any RFID number: None).
        """
        # Check and update assigned RFID object.
        # TODO have a simulation version for this... for testing...
        # Publish assigned RFID object number.
        print("Checking for RFID/NFC card...")
        # Check if a card is available to read
        try:
            uid = self.pn532.read_passive_target(timeout=0.5)
        except:
            print("Card read error - wrong card type used?")
            self.person_id = 0
        print(".", end="")
        if uid is None:
            print("No card found.")
            self.person_id = 0
        else:
            uid_str = ''.join([str(i) for i in uid])
            uid_int = int(uid_str)
            print("Found card with UID: ", uid_int)
            self.person_id = uid_int
        for i in range(5):
            self.pi_person_updates_publisher.publish(pi_id=self.pi_id, person_id=self.person_id)

    def text_to_speech(self, to_speak): # TODO need to use something local probably... ElevenLabs? Coqui TTS?
        speech_file_path = "mp3/speech.mp3"
        response = self.client.audio.speech.create(
            model="tts-1", #tts-1 =  lowest latency, lower quality, tts-1-hd = higher quality, high latency
            voice="alloy",
            input=to_speak
        )
        response.stream_to_file(speech_file_path)

    def speak(self):
        playsound("mp3/speech.mp3")



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



    


    

### Simulated pi node for running simulation of one group on computer ###
# Keyboard inputs for adding and removing people to pis #

# Up to 9 heads in the group #
# Press a number to access that head #
# If there is already a card there, pressing the number will remove the card #
# If there is no card there, then asked to select a person to add from a dropdown menu #

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import wave, os, random
from piper.voice import PiperVoice

from community_interfaces.msg import (
    PiPersonUpdates,
    PiSpeechComplete,
    PiSpeechRequest,
    GroupInfo,
    SimPiPersonAssign
)


class SimPiNode(Node): #TODO

    def __init__(self):
        super().__init__('pi_node')

        # Get pi_id from launch file
        self.declare_parameter('pi_id', 0)
        self.pi_id = self.get_parameter('pi_id').get_parameter_value().integer_value

        # The ID for the RFID object places on the pi (this changes).
        self.person_id = 0 # 0 means no card there

        # Seq list for receiving speech requests - one item for each group
        self.speech_seq = None

        # Initialise publishers
        self.pi_person_updates_publisher = self.create_publisher(PiPersonUpdates, 'pi_person_updates', 10)
        self.pi_speech_complete_publisher = self.create_publisher(PiSpeechComplete, 'pi_speech_complete', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback

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

    def sim_pi_person_assign_callback(self, msg):
        """
        Update what person is on this simulated pi node.
        """
        if msg.pi_id == self.pi_id and msg.person_id != self.person_id:
            self.person_id = msg.person_id
            # Say hello
            if msg.person_id != 0:
                self.text_to_speech(random.choice(self.hello_list), msg.voice_id)


    def group_info_callback(self, msg):
        """
        Used just after initialisation to set the number of people
        in this group.
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
        if self.speech_seq != None:
            # Check if the message is for THIS pi.
            # If yes, submit the text to the speakers.
            if msg.pi_id == self.pi_id and \
            (msg.seq > self.speech_seq[msg.group_id]):
                if self.person_id == msg.person_id: # current person on here must equal the person in the message. 
                    self.get_logger().info(f'Requesting tts for text: {msg.text}')
                    self.text_to_speech(msg.text, msg.voice_id)
                    complete = True
                else: 
                    self.get_logger().info(f'Person for whom speech was requested is not on this pi anymore.')
                    complete = False # need to send a 'not completed' back if person has been removed
                self.pi_speech_complete(
                    complete,
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
                self.speech_seq[msg.group_id] = msg.seq
                

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

    def text_to_speech(self, to_speak, voice_id):
        voicedir = os.path.expanduser('~/Documents/piper/') #Where onnx model files are stored on my machine
        model = voicedir+voice_id
        voice = PiperVoice.load(model)
        wav_file = wave.open('output.wav', 'w')
        text = to_speak
        audio = voice.synthesize(text,wav_file)
        # Play the .wav file
        self.get_logger().info("Playing .wav file") #TODO implement streaming so they can stop talking mid-sentence.
        os.system('aplay output.wav')


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



    


    
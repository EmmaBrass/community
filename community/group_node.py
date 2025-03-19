
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from community_interfaces.msg import (
    GroupInfo
)
from community_interfaces.srv import (
    LlmTextRequest,
    PiSpeechRequest
)
import community.configuration as config
from community.message_type import MessageType
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import os, random, time
import numpy as np
from functools import partial

from pydub.generators import WhiteNoise, Sine

from community.group_convo_manager import GroupConvoManager
from community.helper_functions import HelperFunctions
from community.configuration import PEOPLE_TO_USE

from ament_index_python.packages import get_package_share_directory


class GroupNode(Node):

    def __init__(self):

        super().__init__('group_node')

        # Get group_id from launch file
        self.declare_parameter('group_id', 0)
        self.group_id = self.get_parameter('group_id').get_parameter_value().integer_value

        # Initialise question phase checker
        self.helper = HelperFunctions()

        # Initialise GroupConvoManager object
        self.group_convo_manager = GroupConvoManager(self.group_id)  

        # The number of pis in this group
        self.num_pis = len(config.GROUP_PI_ASSIGNMENTS.get(self.group_id).get('pi_ids'))
        # A list of the pi IDs in the group
        self.pi_ids = config.GROUP_PI_ASSIGNMENTS.get(self.group_id).get('pi_ids') # The pis listed in a constant order - use w/ self.person_pi_dict for tracking completed speech.
        # The people currently in this group (id numbers) (this changes)
        self.group_members = []
        # Simple pi-person dict updated from group_info: used for goodbye text
        self.person_pi_dict = None # for looking up pi from person id
        self.pi_person_dict = None # for looking up person from pi id
        # Flag for if a member has left
        self.left_member_flag = False
        # If very beginning of system, mention question
        self.first_question = True
        # Flag for if last spesch and last text are completed
        self.last_speech_completed = True
        self.last_text_completed = True

        # List of text from person GPTs, things TO SPEAK in FUTURE
        self.speak_list = []
        # List of text that HAS BEEN SENT to the RPi; has a COMPLETE field for if RPi spoke it successfully
        self.spoken_list = []

        # Speak list for chaos phase - one list for each pi.
        self.chaos_speak_list = {pi_id: [] for pi_id in self.pi_ids}
        # Flag for reseting pi service clients when we enter the chaos phase.
        self.chaos_pi_reset_completed = False
        # Dict of flags for if last speech completed in chaos phase.
        self.chaos_last_text_completed = {pi_id: True for pi_id in self.pi_ids}
        self.chaos_last_speech_completed = {pi_id: True for pi_id in self.pi_ids}

        # Some random goodbye phrases to say to fill space
        self.goodbye_list = [
            "Oh someone left, see you later, shame you could not stay.",
            "We have lost someone, this keeps happening, goodbye!",
            "A fellow group member has left us, that is sad.",
            "Ah, goodbye, friend! Come back soon.",
            "Hasta la vista baby",
            "Ah we have lost a group member! In a while, crocodile",
            "Oh someone left, see you later alligator."
        ]

        # Place for static sounds files
        self.static_output_directory = "./static_sounds"
        os.makedirs(self.static_output_directory, exist_ok=True)

        # Initialise subscriber
        group_info_callback_group = MutuallyExclusiveCallbackGroup()
        self.group_info_subscription = self.create_subscription(
            GroupInfo,
            'group_info', 
            self.group_info_callback, 
            10,
            callback_group=group_info_callback_group
        )

        # Dictionaries to store person llm service clients, and callback groups
        self.llm_text_request_clients = {}
        self.llm_callback_groups = {}
        # Create a service client and callback group for all person_ids to use
        for person_id in PEOPLE_TO_USE:
            # Unique service name per Person
            llm_text_service_name = f'llm_text_request_{person_id}' 
            # Create a separate MutuallyExclusiveCallbackGroup for each client
            self.llm_callback_groups[person_id] = MutuallyExclusiveCallbackGroup()
            # Create service clients bound to these callback groups
            llm_text_request_client = self.create_client(
                LlmTextRequest, 
                llm_text_service_name, 
                callback_group=self.llm_callback_groups[person_id]
            )
            # Store references
            self.llm_text_request_clients[person_id] = llm_text_request_client
            # Wait for the services to be available
            while not llm_text_request_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {llm_text_service_name} not available, waiting...')

        # Initialise service clients

        # Dictionaries to store pi service clients, and one callback group for all
        self.pi_speech_request_clients = {}
        self.pi_service_status = {pi_id: False for pi_id in self.pi_ids}
        self.pi_speech_request_callback_group = MutuallyExclusiveCallbackGroup()
        # Create a service client and callback group for each pi in this group
        for pi_id in self.pi_ids:
            # Unique service name per Pi
            service_name = f'pi_speech_request_{pi_id}'  
            # Create a service client bound to this callback group
            client = self.create_client(
                PiSpeechRequest, 
                service_name, 
                callback_group=self.pi_speech_request_callback_group
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

        # Initialise timer callback
        # Publishing happens within the timer_callback
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(
            timer_period, 
            self.timer_callback,
            callback_group=group_info_callback_group
        )   

    def check_pi_services(self):
        """Check if the Pi services are available and update status."""
        for pi_id, client in self.pi_speech_request_clients.items():
            if client.service_is_ready():
                if not self.pi_service_status[pi_id]:
                    self.get_logger().info(f'Service pi_speech_request_{pi_id} is now available!')
                self.pi_service_status[pi_id] = True
            else:
                if self.pi_service_status[pi_id]:
                    self.get_logger().warning(f'Service pi_speech_request_{pi_id} went offline!')
                self.pi_service_status[pi_id] = False

    def group_info_callback(self, msg):
        """
        Callback function for info on group assignment/members.
        """
        self.get_logger().debug('In group_info_callback')
        if msg.group_id == self.group_id:

            # Update raw pi_person matching dicts (used for goodbye text)
            self.person_pi_dict = dict(zip(msg.person_ids, msg.pi_ids))
            self.pi_person_dict = {pi_id: person_id for person_id, pi_id in self.person_pi_dict.items()}

            # Check for people who have left, if there were > 0 people in the group previously
            if len(self.group_members) != 0:
                for person_id in self.group_members:    
                    if person_id not in msg.person_ids:
                        self.get_logger().info('Someone left the group -----------------------------------------------------------------------')
                        self.left_member_flag == True
                        # Update group members
                        self.group_members.remove(person_id)
                        self.get_logger().info(f'Updated group_members: {self.group_members}')

            # Check for new person added, if there > 0 members of the group in the message
            if np.count_nonzero(msg.person_ids) != 0:
                for person_id in msg.person_ids:
                    if person_id not in self.group_members and person_id != 0:
                        self.get_logger().info('Someone joined the group ----------------------------------------------------------------------')
                        # Update group members
                        self.group_members.append(person_id)
                        self.get_logger().info(f'Updated group_members: {self.group_members}')
                    
    def llm_text(
            self, 
            person_id, 
            pi_id,
            directed_id, 
            last_message,
            last_speaker_id,
            event_id, 
            message_type, 
            question_id, 
            question_phase, 
            mention_question,
            chaos
        ):
        self.get_logger().info("In llm_text")
        request = LlmTextRequest.Request()
        request.person_id = person_id
        request.group_id = self.group_id
        request.message_type = message_type
        request.directed_id = directed_id
        request.last_message = last_message
        request.last_speaker_id = last_speaker_id
        request.question_id = question_id
        request.question_phase = question_phase
        request.event_id = event_id
        request.mention_question = mention_question
        future = self.llm_text_request_clients[person_id].call_async(request)
        future.add_done_callback(partial(self.llm_text_callback, 
                                 person_id=person_id, 
                                 pi_id=pi_id, 
                                 message_type=message_type, 
                                 last_message=last_message,
                                 last_speaker_id=last_speaker_id,
                                 directed_id=directed_id, 
                                 question_id=question_id,
                                 mention_question=mention_question,
                                 chaos=chaos))

    def llm_text_callback(
            self, 
            future: Future, 
            person_id,
            pi_id, 
            message_type,
            last_message,
            last_speaker_id,
            directed_id, 
            question_id, 
            mention_question,
            chaos
            ):
        """
        Callback for the future, that will be called when the request is done.
        Update the other GPT clients with this text as well, before requsting more text!
        """
        response = future.result()
        if self.helper.get_question_phase() < config.CHAOS_QUESTION_PHASE:
            self.speak_list.append({
                'person_id' : person_id,
                'pi_id' : pi_id,
                'group_id' : self.group_id,
                'message_type' : message_type,
                'directed_id' : directed_id,
                'last_message' : last_message,
                'last_speaker_id' : last_speaker_id,
                'question_id' : question_id,
                'mention_question' : mention_question,
                'text' : response.text,
                'gpt_message_id' : response.gpt_message_id
            })
        elif self.helper.get_question_phase() == config.CHAOS_QUESTION_PHASE:
            self.chaos_speak_list[pi_id].append({
                'person_id' : person_id,
                'pi_id' : pi_id,
                'group_id' : self.group_id,
                'message_type' : message_type,
                'directed_id' : directed_id,
                'last_message' : last_message,
                'last_speaker_id' : last_speaker_id,
                'question_id' : question_id,
                'mention_question' : mention_question,
                'text' : response.text,
                'gpt_message_id' : response.gpt_message_id
            })

        if chaos:
            self.chaos_last_text_completed[pi_id] = True 
        else:
            self.last_text_completed = True

        if response.completed == True:
            self.get_logger().info(f"Llm text for person {person_id} completed successfully.")
        else:
            self.get_logger().error(f"Llm text for person {person_id} not completed successfully.")

    def pi_speech_callback(self, future: Future, person_id, pi_id, gpt_message_id, chaos):
        """
        Callback for info that the pi has finished speaking a requested text.
        The message should bemoved to spoken_list when request is SENT, and this just records if the pi completed it fully or not.
        """
        self.get_logger().info('In pi_speech_callback!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        if chaos:
            self.chaos_last_speech_completed[pi_id] = True
        else:
            self.last_speech_completed = True
        # Wait for a second to give some pause between speech instances.
        #time.sleep(1)
        self.get_logger().info("looking for future.result")
        response = future.result()
        self.get_logger().info("future.result found")
        if response.completed == True: # Update the dict to say the message speaking was completed by the pi.
            # find item in spoken list with the same gpt_message_id
            for item in reversed(self.spoken_list):
                if (item['gpt_message_id'] == gpt_message_id and
                item['person_id'] == person_id):
                    # update 'completed' to be True
                    item['completed'] = True
                    break
         
    def timer_callback(self):
        """
        Every timer_period seconds, request new text or speech.
        """
        # Update all the pis: register if they are online or not.
        self.check_pi_services()

        question_phase = self.helper.get_question_phase()
        if question_phase < config.CHAOS_QUESTION_PHASE:
            # Say goodbye to whoever left if left_member_flag is true.
            if len(self.speak_list) == 0 and self.left_member_flag == True and self.last_speech_completed == True:
                self.left_member_flag = False
                self.get_logger().info('Publishing goodbye speech')
                # Choose a random group member.
                speaker = None
                while speaker == None:
                    speaker = random.choice(self.group_members)
                    voice_id = self.helper.get_voice_id(speaker)
                    pi_id = self.person_pi_dict[speaker]
                    if not self.pi_service_status.get(pi_id, False): # if the pi is not online!
                        speaker = None # keep speaker as None.
                # Convert text to .wav audio file bytes.
                request = PiSpeechRequest.Request()
                request.person_id = speaker
                request.pi_id = pi_id
                request.group_id = self.group_id
                request.voice_id = voice_id
                request.color = self.helper.get_color(speaker)
                request.message_type = MessageType.GOODBYE.value
                request.text = ""
                request.gpt_message_id = ""
                request.directed_id = 0
                request.question_id = 0
                request.mention_question = False
                request.audio_data = self.helper.text_to_speech_bytes(random.choice(self.goodbye_list), voice_id, self.group_id, volume=1.0)
                self.last_speech_completed = False
                future = self.pi_speech_request_clients[pi_id].call_async(request)
                future.add_done_callback(partial(self.pi_speech_callback,
                                        person_id=speaker,
                                        pi_id=pi_id,
                                        gpt_message_id="",
                                        chaos=False))
            # Check if new speech required (if last person's speech has been spoken).
            # Send a request to the Pi to SPEAK.
            elif len(self.speak_list) == 1 and self.last_speech_completed == True:
                self.get_logger().info('Publishing standard speech')
                # Use the FIRST item in speak_list.
                text_dict = self.speak_list.pop(0)
                # Move it to the spoken list.
                text_dict['completed'] = False
                self.spoken_list.append(text_dict)
                voice_id = self.helper.get_voice_id(text_dict['person_id'])
                # Convert text to .wav audio file bytes.
                # Double check the person is still in the group.
                # Also check if the pi is still online.
                if text_dict['person_id'] in self.group_members and self.pi_service_status.get(text_dict['pi_id'], False) == True:
                    request = PiSpeechRequest.Request()
                    request.person_id = text_dict['person_id']
                    request.pi_id = text_dict['pi_id']
                    request.group_id = self.group_id
                    request.voice_id = voice_id
                    request.color = self.helper.get_color(text_dict['person_id'])
                    request.message_type = text_dict['message_type']
                    request.text = text_dict['text']
                    request.gpt_message_id = text_dict['gpt_message_id']
                    request.directed_id = text_dict['directed_id']
                    request.question_id = text_dict['question_id']
                    request.mention_question = text_dict['mention_question']
                    request.audio_data = self.helper.text_to_speech_bytes(text_dict['text'], voice_id, self.group_id, volume=1.0)
                    self.last_speech_completed = False
                    future = self.pi_speech_request_clients[text_dict['pi_id']].call_async(request) #TODO here... nothing else should be allowed to happen until speech is complete!
                    # otherwise, 'complete' seems to be sent while it's looking elsewhere, and then 
                    # the thing does not register the completed speech and gets stuck...
                    future.add_done_callback(partial(self.pi_speech_callback,
                                            person_id=text_dict['person_id'],
                                            pi_id=text_dict['pi_id'],
                                            gpt_message_id=text_dict['gpt_message_id'],
                                            chaos=False))
                else:
                    self.get_logger().error("Person who should speak is not in group currently!")
            elif len(self.speak_list) == 2:
                self.get_logger().error("Speak list has len of two but it should not!")
            else:
                self.get_logger().debug("Nothing to speak right now.")

            if len(self.group_members) > 0 and self.last_text_completed == True and len(self.speak_list) == 0:
                self.get_logger().info("Requesting text")
                if len(self.spoken_list) != 0:
                    self.get_logger().info(f"Last 2 items in spoken list: {str(self.spoken_list[-2:])}")
                    last_item = self.spoken_list[-1]  # Get the last item in the list
                    if self.spoken_list[-1]['person_id'] not in self.group_members:
                        last_message = "" # Don't use the last message if that person has left the group!
                    else:
                        last_message = self.spoken_list[-1]['text'] # Get last message
                    if len(self.spoken_list) > 1:
                        second_last_item = self.spoken_list[-2]
                    else:
                        second_last_item = None
                else:
                    last_item = None
                    second_last_item = None
                    last_message = ""
                last_speaker_id, person_id, message_type, directed_id, event_id, question_id, question_phase = self.group_convo_manager.get_next(
                    self.group_members, 
                    last_item,
                    second_last_item
                )
                self.get_logger().info("last_speaker_id")
                self.get_logger().info(str(last_speaker_id))
                self.get_logger().info("person_id (person to speak next)")
                self.get_logger().info(str(person_id))
                self.get_logger().info("Name of person to speak next")
                self.get_logger().info(str(self.helper.get_name(person_id)))
                self.get_logger().info("message_type")
                self.get_logger().info(str(MessageType(message_type).name))
                self.get_logger().info("directed_id")
                self.get_logger().info(str(directed_id))
                self.get_logger().info("event_id")
                self.get_logger().info(str(event_id))
                self.get_logger().info("question_id")
                self.get_logger().info(str(question_id))
                self.get_logger().info("question_phase")
                self.get_logger().info(str(question_phase))

                if message_type == MessageType.SWITCH.value or message_type == MessageType.ALONE.value:
                    mention_question = self.check_last_question_mention(person_id)
                else:
                    mention_question = False
                self.last_text_completed = False
                self.llm_text(
                    person_id, 
                    self.person_pi_dict[person_id],
                    directed_id, 
                    last_message, # last_message is the most recent thing that was said in the group.
                    last_speaker_id,
                    event_id, 
                    message_type, 
                    question_id, 
                    question_phase, 
                    mention_question,
                    False
                )

        elif question_phase >= config.CHAOS_QUESTION_PHASE:

            if self.chaos_pi_reset_completed == False:
                # We need to destroy the pi_services 
                for pi_id in self.pi_ids:
                    # Destroy the current client
                    self.destroy_client(self.pi_speech_request_clients[pi_id])
                # and then recreate them
                # Now, with each one in it's own MutuallyExclusive callback group,
                # to allow them to talk over one another.
                self.pi_speech_request_clients = {}
                self.pi_service_status = {pi_id: False for pi_id in self.pi_ids}
                self.pi_speech_request_callback_groups = {}
                # Create a service client and callback group for each pi in this group
                for pi_id in self.pi_ids:
                    # Unique service name per Pi
                    service_name = f'pi_speech_request_{pi_id}'  
                    # Create callback group
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
                self.chaos_pi_reset_completed = True

            if question_phase == config.CHAOS_QUESTION_PHASE and len(self.group_members) > 0:
                
                for pi in self.pi_ids:
                    # Check if the pi is still online and a person is there:
                    if self.pi_service_status.get(pi, False) and self.pi_person_dict[pi] != 0:
                        # Request text from llm client, if there is a person on the pi!
                        # Check if chaos_last_text_completed = True for this pi
                        # Get current person_id for the pi
                        person_id = self.pi_person_dict[pi]
                        if len(self.chaos_speak_list[pi]) == 0 and self.chaos_last_text_completed[pi] == True:
                            self.chaos_last_text_completed[pi] = False
                            self.llm_text(
                                person_id = person_id, 
                                pi_id = pi,
                                directed_id = 0, 
                                last_message = "",
                                last_speaker_id=0,
                                event_id = 0, 
                                message_type = MessageType.OPEN.value, 
                                question_id = person_id, 
                                question_phase = question_phase, 
                                mention_question = False,
                                chaos = True
                            )
                        # Request speech, for each pi client
                        # Use the FIRST item in speak_list.
                        if len(self.chaos_speak_list[pi]) == 1 and self.chaos_last_speech_completed[pi] == True:
                            text_dict = self.chaos_speak_list[pi].pop(0)
                            # Check the pi still has the same person it had when the speech was created (and they are still in the group)!
                            if person_id == text_dict['person_id'] and person_id in self.group_members:
                                voice_id = self.helper.get_voice_id(person_id)
                                request = PiSpeechRequest.Request()
                                request.person_id = person_id
                                request.pi_id = pi
                                request.group_id = self.group_id
                                request.voice_id = voice_id
                                request.color = self.helper.get_color(person_id)
                                request.message_type = text_dict['message_type']
                                request.text = text_dict['text']
                                request.gpt_message_id = text_dict['gpt_message_id']
                                request.directed_id = text_dict['directed_id']
                                request.question_id = text_dict['question_id']
                                request.mention_question = text_dict['mention_question']
                                request.audio_data = self.helper.text_to_speech_bytes(text_dict['text'], voice_id, person_id, volume=0.65)
                                self.chaos_last_speech_completed[pi] = False
                                future = self.pi_speech_request_clients[pi].call_async(request)
                                future.add_done_callback(partial(self.pi_speech_callback,
                                                        person_id=person_id,
                                                        pi_id=pi,
                                                        gpt_message_id=text_dict['gpt_message_id'],
                                                        chaos=True))
                            else:
                                self.get_logger().info("Person is not in group or not on same pi anymore !")
                        elif len(self.chaos_speak_list[pi]) > 1:
                            self.get_logger().warn("Chaos to speak list is too long (>1)!")


            # If static phase, need to request just random static 'speech' from each pi.
            elif question_phase == config.STATIC_QUESTION_PHASE and len(self.group_members) > 0:
                for pi in self.pi_ids:
                    # Check if the pi is still online:
                    if self.pi_service_status.get(pi, False) and self.chaos_last_speech_completed[pi] == True and self.pi_person_dict[pi] != 0:
                        # Get current person_id for the pi
                        person_id = self.pi_person_dict[pi]
                        request = PiSpeechRequest.Request()
                        request.person_id = person_id
                        request.pi_id = self.person_pi_dict[person_id]
                        request.group_id = self.group_id
                        request.voice_id = self.helper.get_voice_id(person_id)
                        request.color = [210,0,0]
                        request.message_type = MessageType.OPEN.value
                        request.text = "STATIC"
                        request.gpt_message_id = ""
                        request.directed_id = 0
                        request.question_id = 0
                        request.mention_question = False
                        request.audio_data = self.generate_static_sounds(output_dir=self.static_output_directory, duration=6, volume=0.25)
                        self.chaos_last_speech_completed[pi] = False
                        future = self.pi_speech_request_clients[pi].call_async(request)
                        future.add_done_callback(partial(self.pi_speech_callback,
                                                person_id=person_id,
                                                pi_id=self.person_pi_dict[person_id],
                                                gpt_message_id="",
                                                chaos=True))

    def check_last_question_mention(self, person_id):
        """
        Check if this person explicitly mentioned their question recently.
        """
        mention_question = False

        if len(self.spoken_list) > 0:
            reversed_spoken_list = self.spoken_list[::-1]
            len_from_end = -1
            for num, item in enumerate(reversed_spoken_list):
                # Find the last item where this person mentioned their question
                if item['mention_question'] == True and item['person_id'] == person_id:
                    len_from_end = num
                    break
            if len_from_end > config.MIN_QUESTION_MENTION or len_from_end == -1: # If -1, then their question has not been mentioned at all in spoken list for this group.
                mention_question = True
        else: 
            mention_question = True
 
        return mention_question
    
    def generate_static_sounds(self, output_dir, duration, volume=1.0):
        """
        Generate varied electronic static audio files with white noise and modulation.

        :param output_dir: Directory to save the generated audio files.
        :param duration: Duration of each audio file in seconds.
        :param volume: Volume scaling factor (1.0 = full volume, 0.5 = 50%, 0.1 = 10%, etc.)

        :returns: The audio data as a list of bytes.
        """
        # Ensure volume is in range [0, 1]
        volume = max(0, min(volume, 1))

        # Convert volume scaling factor to decibels
        volume_db = 20 * np.log10(volume) if volume > 0 else -100  # -100 dB = near silence

        # Generate base white noise
        noise = WhiteNoise().to_audio_segment(duration * 1000)  # Duration in milliseconds

        # Add random amplitude modulation (low-frequency sine wave)
        mod_freq = np.random.uniform(0.1, 5)  # Modulation frequency in Hz
        mod_depth = np.random.uniform(-15, -5)  # Modulation depth in dB
        sine_wave = Sine(mod_freq).to_audio_segment(duration * 1000).apply_gain(mod_depth)
        modulated_noise = noise.overlay(sine_wave)

        # Apply random low-pass and high-pass filters
        low_cutoff = np.random.uniform(500, 5000)  # Low-pass cutoff frequency
        high_cutoff = np.random.uniform(50, 400)   # High-pass cutoff frequency
        filtered_noise = modulated_noise.low_pass_filter(low_cutoff).high_pass_filter(high_cutoff)

        # Add slight random gain for variation
        final_noise = filtered_noise.apply_gain(np.random.uniform(-4, 4))

         # Apply user-defined volume scaling
        final_noise = final_noise.apply_gain(volume_db)

        # Export to file
        wav_file = f"{output_dir}/static_sound.wav"
        final_noise.export(wav_file, format="wav")
        print(f"Generated: {wav_file}")

        # Convert audio to bytes
        with open(wav_file, 'rb') as f:
            audio_data = f.read()
            audio_uint8 = list(audio_data)
            
        return audio_uint8
                
            

def main(args=None):
    rclpy.init(args=args)

    group_node = GroupNode()
    # Use MultiThreadedExecutor to allow parallel execution
    executor = MultiThreadedExecutor()

    rclpy.spin(group_node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    group_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



    


    
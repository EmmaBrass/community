
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from community_interfaces.msg import (
    GroupInfo
)
from community_interfaces.srv import (
    LlmTextRequest,
    LlmRewindRequest,
    LlmUpdateRequest,
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
        self.group_convo_manager = GroupConvoManager()

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
        # Track total of text requests sent for this group
        self.text_requests = 0
        # Flag for if last spesch and last text are completed
        self.last_speech_completed = True
        self.last_text_completed = True

        self.llm_other_update_completed = {}

        # Speak list for chaos phase - one list for each pi.
        self.chaos_speak_list = {pi_id: [] for pi_id in self.pi_ids}

        # List of text from person GPTs, things TO SPEAK in FUTURE
        self.speak_list = []
        # List of text that HAS BEEN SENT to the RPi; has a COMPLETE field for if RPi spoke it successfully
        self.spoken_list = []

        # Some random goodbye phrases to say to fill space
        self.goodbye_list = [
            "Oh someone left, see you later, shame you could not stay.",
            "We have lost someone, this keeps happeneind, goodbye!",
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

        # Dictionaries to store service clients, and callback groups
        self.llm_text_request_clients = {}
        self.llm_rewind_request_clients = {}
        self.llm_update_request_clients = {}
        self.llm_callback_groups = {}
        # Create a service client and callback group for all person_ids
        for person_id, _ in self.helper.people_data.items():
            # Unique service name per Person
            llm_text_service_name = f'llm_text_request_{person_id}' 
            llm_rewind_service_name = f'llm_rewind_request_{person_id}'  
            llm_update_service_name = f'llm_update_request_{person_id}' 
            # Create a separate MutuallyExclusiveCallbackGroup for each client
            self.llm_callback_groups[person_id] = MutuallyExclusiveCallbackGroup()
            # Create service clients bound to these callback group
            llm_text_request_client = self.create_client(
                LlmTextRequest, 
                llm_text_service_name, 
                callback_group=self.llm_callback_groups[person_id]
            )
            llm_rewind_request_client = self.create_client(
                LlmRewindRequest, 
                llm_rewind_service_name, 
                callback_group=self.llm_callback_groups[person_id]
            )
            llm_update_request_client = self.create_client(
                LlmUpdateRequest, 
                llm_update_service_name, 
                callback_group=self.llm_callback_groups[person_id]
            )
            # Store references
            self.llm_text_request_clients[person_id] = llm_text_request_client
            self.llm_rewind_request_clients[person_id] = llm_rewind_request_client
            self.llm_update_request_clients[person_id] = llm_update_request_client
            # Wait for the services to be available
            while not llm_text_request_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {llm_text_service_name} not available, waiting...')
            while not llm_rewind_request_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {llm_rewind_service_name} not available, waiting...')
            while not llm_update_request_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {llm_update_service_name} not available, waiting...') 

        # Initialise service clients

        # Dictionaries to store service clients, and one callback group for all
        self.pi_speech_request_clients = {}
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
            # Wait for the service to be available
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {service_name} not available, waiting...')

        # Initialise timer callback
        # Publishing happens within the timer_callback
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(
            timer_period, 
            self.timer_callback,
            callback_group=group_info_callback_group
        )   


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
                        self.get_logger().info('Someone left the group')
                        # Update all others in the group
                        self.group_members.remove(person_id)
                        self.get_logger().info(f'Updated group_members: {self.group_members}')
                        # Service for llm update
                        self.llm_update("left", person_id)
                        self.get_logger().info("HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEErrrrrrrrrrrrrrrrrrrrrrrrrrrrrr")
                        self.get_logger().info(str(self.helper.get_question_phase()))
                        self.get_logger().info(str(config.CHAOS_QUESTION_PHASE))
                        # Rewind speak list as group makeup has changed, if not in chaos phase
                        if self.helper.get_question_phase() < config.CHAOS_QUESTION_PHASE:
                            self.get_logger().info("HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEeeeeeeeeeeeeeeeeee")
                            self.llm_rewind(False, person_id)

            # Check for new person added, if there > 0 members of the group in the message
            if np.count_nonzero(msg.person_ids) != 0:
                for person_id in msg.person_ids:
                    if person_id not in self.group_members and person_id != 0:
                        self.get_logger().info('Someone joined the group')
                        # Service for llm update
                        self.llm_update("joined", person_id)
                        self.group_members.append(person_id)
                        self.get_logger().info(f'Updated group_members: {self.group_members}')
                        self.get_logger().info("HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEeeeeeeeeeeeeeeeeee")
                        self.get_logger().info(str(self.helper.get_question_phase()))
                        self.get_logger().info(str(config.CHAOS_QUESTION_PHASE))
                        # Rewind speak list as group makeup has changed, if not in chaos phase
                        if self.helper.get_question_phase() < config.CHAOS_QUESTION_PHASE:
                            self.llm_rewind(True, person_id)

    def llm_update(self, update_type, person_id, text=""):
        """
        Send service request for llm update for all people in the group.

        :param type: The type of update: "left" (someone else left), "joined" (someone else joined), 
        "new" (this person is in a new group), or "other" (someone else spoke).
        :param person_id: The id of the person who left/joined/spoke whatever; the focus person.
        """
        if update_type == "left":
            for pid in (p for p in self.group_members if p != person_id):
                request = LlmUpdateRequest.Request()
                request.type = "left"
                request.person_id = person_id
                request.group_id = self.group_id
                request.group_members = self.group_members
                request.text = text
                future = self.llm_update_request_clients[pid].call_async(request)
                future.add_done_callback(lambda f, pid=pid: self.llm_update_callback(f, pid))
        elif update_type == "joined":
            # Send a seperate one to the person who has joined ! -> to tell them they are in a new group. 
            request = LlmUpdateRequest.Request()
            request.type = "new"
            request.person_id = person_id
            request.group_id = self.group_id
            request.group_members = self.group_members
            request.text = text
            future = self.llm_update_request_clients[person_id].call_async(request)
            future.add_done_callback(lambda f, pid=person_id: self.llm_update_callback(f, pid))
            # Tell everyone else they have joined.
            for pid in (p for p in self.group_members if p != person_id):
                request = LlmUpdateRequest.Request()
                request.type = "joined"
                request.person_id = person_id
                request.group_id = self.group_id
                request.group_members = self.group_members
                request.text = text
                future = self.llm_update_request_clients[pid].call_async(request)
                future.add_done_callback(lambda f, pid=pid: self.llm_update_callback(f, pid))
        elif update_type == "other":
            self.get_logger().info("HERE IN LLM UPDATE !!!!!!")
            # Tell everyone else what the focus person has said.
            # Make and array for checking response to this;
            # when responses are back from ALL lllm clients, then we can set 
            # last_text_completed flag to be True.
            self.llm_other_update_completed = {}
            for pid in (p for p in self.group_members if p != person_id):
                self.llm_other_update_completed[pid] = False
                request = LlmUpdateRequest.Request()
                request.type = "other"
                request.person_id = person_id
                request.group_id = self.group_id
                request.group_members = self.group_members
                request.text = text
                future = self.llm_update_request_clients[pid].call_async(request)
                future.add_done_callback(lambda f, pid=pid, update_type="other": self.llm_update_callback(f, pid, update_type))
            # Still set the self.last_text_completed flat to True if there was noone to send an update to
            if len(self.llm_other_update_completed) == 0:
                self.last_text_completed = True

    def llm_update_callback(self, future: Future, person_id, update_type=""):
        """
        Callback for the future, that will be called when the request is done.
        """
        response = future.result()
        if response.completed == True:
            if update_type == "other":
                self.llm_other_update_completed[person_id] = True
            self.get_logger().info(f"Llm update for person {person_id} completed.")
        else:
            self.get_logger().info(f"Llm update  for person {person_id} not completed.")
        if update_type == "other" and all(self.llm_other_update_completed.values()) == True:
            self.last_text_completed = True # last text received AND all other LLMs in group updated about it.

    def llm_rewind(self, joined, person_id, leave=1):
        """
        Extract gpt_message_ids for each new instance of a person_id in self.speak_list.
        Publish a message to delete these gpt messages.
        Ask the RelationshipManagr to rewind to the last spoken tick.
        If someone joins, we rewind but leave the next two items
        If someone leaves, we rewind to but leave two UNLESS the person who left is in those two,
        in which case we rewind until the person who left is no longer in the list.

        :param joined: True if someone has joined the group, False if someone has left the group.
        :param person_id: The person who left or joined.
        :param leave: The number of messages to leave in the speak list, whilst the rest are deleted.
        """
        # Only do anything if there are some unpublished messages in speak_list
        if len(self.speak_list) > 0:
            self.get_logger().info('Removing unspoken messages in speak list.')

            if joined == True:
                # If someone has joined, rewind all but last one message in self.speak_list
                to_delete = self.speak_list[leave:]
                self.speak_list = self.speak_list[:leave]
            else:
                # If someone has left, rewind but leave one UNLESS person who left is in that one, then rewind further
                to_delete = self.speak_list
                new_speak_list = []
                for item in self.speak_list[:leave]:
                    if item['person_id'] != person_id: # if person to speak is NOT the one who has just left, add to new_speak_list
                        new_speak_list.append(item)
                        del to_delete[0]
                    else:
                        break # stop building the new_speak_list
                self.speak_list = new_speak_list
                if len(self.speak_list) == 0:
                    self.left_member_flag = True # So that someone will say 'goodbye' to fill the space.
                
            self.get_logger().info('Just reduced speak_list')
            
            # Collect unique person_id entries from the right side of the list, for deletion
            unique_person_ids = set()  # To store the unique person_ids
            to_rewind = []  # To store the results (person_id and gpt_message_id)

            for entry in to_delete:
                person_id = entry['person_id']
                if person_id not in unique_person_ids and entry['gpt_message_id'] != "":
                    # If it's a new person_id, save the person_id and gpt_message_id
                    unique_person_ids.add(person_id)
                    to_rewind.append({
                        'person_id': person_id,
                        'gpt_message_id': entry['gpt_message_id']
                    })

            # Request an llm rewind for each item in to_rewind list
            # TODO this does not work perfectly because it only rewinds to the last
            # time that a specific person spoke, doesn't rewind ALL messages including the
            # ones given to it where someone else has spoken !
            self.get_logger().info('Sending an llm rewind request for each item in result list.')
            for item in to_rewind:
                request = LlmRewindRequest.Request()
                pid = item['person_id']
                request.person_id = pid
                request.group_id = self.group_id
                request.gpt_message_id = item['gpt_message_id']
                future = self.llm_rewind_request_clients[item['person_id']].call_async(request)
                future.add_done_callback(lambda f, pid=pid: self.llm_rewind_callback(f, pid))

    def llm_rewind_callback(self, future: Future, person_id):
        """
        Callback for the future, that will be called when the request is done.
        """
        response = future.result()
        if response.completed == True:
            self.get_logger().info(f"Llm rewind for person {person_id} completed.")
        else:
            self.get_logger().info(f"Llm rewind for person {person_id} not completed.")
                    
    def llm_text(
            self, 
            person_id, 
            pi_id,
            directed_id, 
            event_id, 
            message_type, 
            question_id, 
            question_phase, 
            mention_question
        ):
        self.get_logger().info("In llm_text")
        request = LlmTextRequest.Request()
        request.person_id = person_id
        request.group_id = self.group_id
        request.message_type = message_type
        request.directed_id = directed_id
        request.question_id = question_id
        request.question_phase = question_phase
        request.event_id = event_id
        request.mention_question = mention_question
        future = self.llm_text_request_clients[person_id].call_async(request)
        future.add_done_callback(partial(self.llm_text_callback, 
                                 person_id=person_id, 
                                 pi_id=pi_id, 
                                 message_type=message_type, 
                                 directed_id=directed_id, 
                                 question_id=question_id,
                                 mention_question=mention_question))

    def llm_text_callback(
            self, 
            future: Future, 
            person_id,
            pi_id, 
            message_type,
            directed_id, 
            question_id, 
            mention_question
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
                'question_id' : question_id,
                'mention_question' : mention_question,
                'text' : response.text,
                'gpt_message_id' : response.gpt_message_id
            })

        self.llm_update("other", person_id, response.text)

        if response.completed == True:
            self.get_logger().info(f"Llm text for person {person_id} completed successfully.")
        else:
            self.get_logger().info(f"Llm text for person {person_id} not completed successfully.")

    def pi_speech_callback(self, future: Future, person_id, gpt_message_id):
        """
        Callback for info that the pi has finished speaking a requested text.
        The message should bemoved to spoken_list when request is SENT, and this just records if the pi completed it fully or not.
        """
        self.get_logger().info('In pi_speech_callback!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        self.last_speech_completed = True
        # Wait for a second to give some pause between speech instances.
        time.sleep(1)
        response = future.result()
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
        question_phase = self.helper.get_question_phase()
        if question_phase < config.CHAOS_QUESTION_PHASE:
            # Say goodbye to whoever left if left_member_flag is true.
            if len(self.speak_list) == 0 and self.left_member_flag == True and self.last_speech_completed == True:
                self.left_member_flag = False
                self.get_logger().info('Publishing goodbye speech')
                # Choose a random group member.
                speaker = random.choice(self.group_members)
                voice_id = self.helper.get_voice_id(speaker)
                pi_id = self.person_pi_dict[speaker]
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
                request.audio_data = self.helper.text_to_speech_bytes(random.choice(self.goodbye_list), voice_id, self.group_id)
                self.last_speech_completed = False
                future = self.pi_speech_request_clients[pi_id].call_async(request)
                future.add_done_callback(partial(self.pi_speech_callback,
                                        person_id=speaker,
                                        gpt_message_id=""))

            # Check if new speech required (if last person's speech has been spoken).
            # Send a request to the Pi to SPEAK.
            elif len(self.speak_list) != 0 and self.last_speech_completed == True:
                self.get_logger().info('Publishing standard speech')
                # Use the FIRST item in speak_list.
                text_dict = self.speak_list.pop(0)
                # Move it to the spoken list.
                text_dict['completed'] = False
                self.spoken_list.append(text_dict)
                voice_id = self.helper.get_voice_id(text_dict['person_id'])
                # Convert text to .wav audio file bytes.
                # Double check the person is still in the group.
                if text_dict['person_id'] in self.group_members:
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
                    request.audio_data = self.helper.text_to_speech_bytes(text_dict['text'], voice_id, self.group_id)
                    self.last_speech_completed = False
                    future = self.pi_speech_request_clients[text_dict['pi_id']].call_async(request)
                    future.add_done_callback(partial(self.pi_speech_callback,
                                        person_id=text_dict['person_id'],
                                        gpt_message_id=text_dict['gpt_message_id']))
                else:
                    self.get_logger().error("Person who should speak is not in group currently!")

            # Backlog of things in to_speak list + text requests not replied to yet.
            speak_backlog = self.text_requests - len(self.spoken_list)

            if len(self.group_members) > 0 and speak_backlog < config.MAX_SPEAK_BACKLOG_LEN and self.last_text_completed == True:
                self.get_logger().info("Requesting text")
                self.get_logger().info(f"Length of speak list: {str(len(self.speak_list))}")
                if len(self.speak_list) != 0:
                    # Get last_speaker, second_last_speaker, and last_message_directed from speech list
                    last_item = self.speak_list[-1]  # Get the last item in the list
                    last_speaker = last_item['person_id']
                    last_message_directed = last_item['directed_id']
                    last_question_id = last_item['question_id']
                    if len(self.speak_list) > 1:
                        second_last_item = self.speak_list[-2]
                        second_last_speaker = second_last_item['person_id']
                    else:
                        second_last_speaker = 0
                elif len(self.spoken_list) != 0:
                    self.get_logger().info(f"Spoken list: {str(self.spoken_list)}")
                    # Group has reset in some way - get from spoken list.
                    last_item = self.spoken_list[-1]  # Get the last item in the list
                    last_speaker = last_item['person_id']
                    last_message_directed = last_item['directed_id']
                    last_question_id = last_item['question_id']
                    if len(self.spoken_list) > 1:
                        second_last_item = self.spoken_list[-2]
                        second_last_speaker = second_last_item['person_id']
                    else:
                        second_last_speaker = 0
                else:
                    last_speaker = 0
                    last_message_directed = 0
                    second_last_speaker = 0
                    last_question_id = 0
                person_id, message_type, directed_id, event_id, question_id, question_phase = self.group_convo_manager.get_next(
                    self.group_members, 
                    last_speaker, 
                    second_last_speaker, 
                    last_question_id,
                    last_message_directed
                )
                if message_type == MessageType.SWITCH.value or message_type == MessageType.ALONE.value:
                    mention_question = self.check_last_question_mention(person_id)
                else:
                    mention_question = False
                self.last_text_completed = False
                self.llm_text(
                    person_id, 
                    self.person_pi_dict[person_id],
                    directed_id, 
                    event_id, 
                    message_type, 
                    question_id, 
                    question_phase, 
                    mention_question
                )
                self.text_requests += 1

        elif question_phase >= config.CHAOS_QUESTION_PHASE:

            # We need to destroy the pi_services 
            for pi_id in self.pi_ids:
                # Destroy the current client
                self.destroy_client(self.pi_speech_request_clients[pi_id])
            # and then recreate them
            # Now, with each one in it's own MutuallyExclusive callback group,
            # to allow them to talk over one another.
            # Dictionaries to store service clients, and one callback group for all
            self.pi_speech_request_clients = {}
            self.pi_speech_request_callback_groups = {}
            # Create a service client and callback group for each pi in this group
            for pi_id in self.pi_ids:
                # Unique service name per Pi
                service_name = f'pi_speech_request_{pi_id}'  
                # Create a separate MutuallyExclusiveCallbackGroup for each client
                self.pi_speech_request_callback_groups[pi_id] = MutuallyExclusiveCallbackGroup()
                # Create a service client bound to this callback group
                client = self.create_client(
                    PiSpeechRequest,
                    service_name, 
                    callback_group=self.pi_speech_request_callback_group
                )
                # Store references
                self.pi_speech_request_clients[pi_id] = client
                # Wait for the service to be available
                while not client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info(f'Service {service_name} not available, waiting...')

            if question_phase == config.CHAOS_QUESTION_PHASE and len(self.group_members) > 0:
                
                for pi in self.pi_ids:
                    # Request text from llm client
                    if len(self.chaos_speak_list[pi]) < config.MAX_SPEAK_LIST_LEN:
                        # Get current person_id for the pi
                        person_id = self.pi_person_dict[pi]
                        self.llm_text(
                            person_id = person_id, 
                            pi_id = pi,
                            directed_id = 0, 
                            event_id = 0, 
                            message_type = MessageType.OPEN.value, 
                            question_id = person_id, 
                            question_phase = question_phase, 
                            mention_question = False
                        )
                    # Request speech, for each pi client
                    # Get current person_id for the pi
                    person_id = self.pi_person_dict[pi]
                    # Use the FIRST item in speak_list.
                    text_dict = self.chaos_speak_list[pi].pop(0)
                    # Check the pi still has the same person it had when the speech was created (and they are still in the group)!
                    if person_id == text_dict['person_id'] and person_id in self.group_members:
                        request = PiSpeechRequest.Request()
                        request.person_id = person_id
                        request.pi_id = pi
                        request.group_id = self.group_id
                        request.voice_id = self.helper.get_voice_id(person_id)
                        request.color = self.helper.get_color(person_id)
                        request.message_type = text_dict['message_type']
                        request.text = text_dict['text']
                        request.gpt_message_id = text_dict['gpt_message_id']
                        request.directed_id = text_dict['directed_id']
                        request.question_id = text_dict['question_id']
                        request.mention_question = text_dict['mention_question']
                        request.audio_data = self.helper.text_to_speech_bytes(text_dict['text'], voice_id, person_id)
                        future = self.pi_speech_request_clients[person_id].call_async(request)
                        future.add_done_callback(partial(self.pi_speech_callback,
                                                person_id=person_id,
                                                gpt_message_id=text_dict['gpt_message_id']))
                    else:
                        self.get_logger().info("Person is not in group or not on same pi anymore !")

            # If static phase, need to request just random static 'speech' from each pi.
            elif question_phase == config.STATIC_QUESTION_PHASE and len(self.group_members) > 0:
                for pi in self.pi_ids:
                    # Get current person_id for the pi
                    person_id = self.pi_person_dict[pi]
                    request = PiSpeechRequest.Request()
                    request.person_id = person_id
                    request.pi_id = self.person_pi_dict[speaker]
                    request.group_id = self.group_id
                    request.voice_id = self.helper.get_voice_id(person_id)
                    request.color = self.helper.get_color(person_id)
                    request.message_type = MessageType.OPEN.value
                    request.text = "STATIC"
                    request.gpt_message_id = ""
                    request.directed_id = 0
                    request.question_id = 0
                    request.mention_question = False
                    request.audio_data = self.generate_static_sounds(output_dir=self.static_output_directory, duration=6)
                    future = self.pi_speech_request_clients[speaker].call_async(request)
                    future.add_done_callback(partial(self.pi_speech_callback,
                                            person_id=person_id,
                                            gpt_message_id=""))

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
        if self.first_question == True:
            mention_question = True
            self.first_question = False
 
        return mention_question
    
    def generate_static_sounds(self, output_dir, duration):
        """
        Generate varied electronic static audio files with white noise and modulation.

        :param output_dir: Directory to save the generated audio files.
        :param duration: Duration of each audio file in seconds.

        :returns: The audio data as a list of bytes.
        """
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
        final_noise = filtered_noise.apply_gain(np.random.uniform(-5, 10))

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



    


    
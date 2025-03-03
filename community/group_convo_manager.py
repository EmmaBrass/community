import community.configuration as config
from community.message_type import MessageType
from community.helper_functions import HelperFunctions

from ament_index_python.packages import get_package_share_directory

import random, time, os, yaml

class GroupConvoManager():
    """
    Manages who speaks next in a conversation within a group.
    """

    # Will need to set it so that if that group member leaves then we move away from their question

    def __init__(self):
        # Keep track of how long a convo has been back and forth between two people
        self.back_and_forth_counter = 0
        
        # Helper functions
        self.helper = HelperFunctions()

        # Keep track of which events this group has already discussed
        self.discussed_event_ids = []

        # Flag just to ensure FIRST ever thing spoken in a group is a new question
        self.first_question_flag = True

    # Function to convert hours, minutes, and seconds to total seconds
    def convert_to_seconds(self, timestamp):
        hours = timestamp.get('hours', 0)
        minutes = timestamp.get('minutes', 0)
        seconds = timestamp.get('seconds', 0)
        total_seconds = (hours * 3600) + (minutes * 60) + seconds
        return total_seconds

    def event_checker(self):
        """
        Check if elapsed time is more than event timestamp for each event, in seconds.
        If yes, check if that event has already been talked about.
        If it has not been talked about, return the event_id.
        """
        if config.EVENTS == True: # only check if events turned on in config file
            time_now = time.time()
            elapsed_seconds = time_now - self.initialise_time
            # Check if the current time is past the event time
            for event_id, event in self.helper.events_data.items():
                timestamp = event['timestamp']
                timestamp_seconds = self.convert_to_seconds(timestamp)
                if elapsed_seconds > timestamp_seconds and elapsed_seconds < (timestamp_seconds + config.MAX_EVENT_DISCUSS_WAIT):
                    # Check if that event has already been discussed by this group
                    if event_id not in self.discussed_event_ids:
                        self.discussed_event_ids.append(event_id)
                        return int(event_id)  # Return event_id as integer
        return 0  # Returns 0 if no event to talk about, or events turned off in config file
    
    
    def get_next(self, group_members, last_item, second_last_item):
        """
        Get next speaker and speech type.

        :param group_members: Current group members.
        :param last_item: The most recent spoken thing to consider.
        :param second_last_item: The second most recent spoken thing to consider.

        :returns next_speaker: The next group member to talk
        :returns message_type: What type of thing they are going to say.
        :returns direct_to: The person the next message should be directed at (0 if none)
        :returns event_id: If of event to be talked about (0 if none)
        """

        if last_item != None:
            last_speaker = last_item['person_id'] if last_item['person_id'] != 0 else None # Who spoke most recently
            last_message_directed = last_item['directed_id'] if last_item['directed_id'] != 0 else None # Who the last message was directed at, if anyone.
            last_question_id = last_item['question_id'] if last_item['question_id'] != 0 else None # The ID of the person whose question is currently being discussed.
            last_message_type = last_item['message_type']  if last_item['message_type'] != 0 else None # The message type of the most recent message (int)
        else:
            last_speaker = None
            last_message_directed = None
            last_question_id = None
            last_message_type = None
        if second_last_item != None:
            second_last_speaker = last_item['person_id']
        else:
            second_last_speaker = None

        # directed_id is 0 (noone) by default (message not directed at anyone)
        directed_id = 0
        # event_id is 0 by default (no event to speak about)
        event_id = 0

        # By default, question_id for next speech is the same as the last one
        question_id = last_question_id

        question_phase = self.helper.get_question_phase() #TODO this will AFFECT the message type 
        # -> e.g. cannot have SWITCH as a message type if we have moved on to question phase 5 onwards.
        # In phase 4, there should be more of people asking their own questions.
        if question_phase < config.CHAOS_QUESTION_PHASE-1:
            switch_percent = config.SWITCH_PERCENT
        else:
            switch_percent = config.SWITCH_PERCENT + 5

        if self.first_question_flag == True:
            message_type = MessageType.SWITCH.value
            next_speaker = random.choice(group_members)
            question_id = next_speaker
            self.first_question_flag = False

        if len(group_members) == 1:
            next_speaker = group_members[0]
            event_id = self.event_checker()
            if event_id != 0:
                message_type = MessageType.EVENT.value
            else: 
                message_type = MessageType.ALONE.value
            question_id = next_speaker
            self.back_and_forth_counter = 0

        elif len(group_members) == 2:
            # Next speaker is person in group_members who is not last_speaker!
            filtered_members = [item for item in group_members if item != last_speaker]
            next_speaker = random.choice(filtered_members)
            directed_id = [item for item in group_members if item != next_speaker][0] # Direct next message to the other person
            event_id = self.event_checker()
            if event_id != 0:
                message_type = MessageType.EVENT.value
            else: 
                rand = random.randint(0, 100)
                if rand <= switch_percent and next_speaker != last_question_id:
                    message_type = MessageType.SWITCH.value
                    question_id = next_speaker
                else:
                    message_type = MessageType.DIRECT.value
            self.back_and_forth_counter = 0 # back and forth counter doesn't apply if only 2 people in the group

        elif len(group_members) > 2:
            interrupt_check = random.randint(0,100)
            if last_speaker == None:
                # Noone has spoken yet - startup of the system
                # Just choose someone random from existing members
                next_speaker = random.choice(group_members)
                event_id = self.event_checker()
                if event_id != 0:
                    message_type = MessageType.EVENT.value
                else: 
                    rand = random.randint(0, 100)
                    if rand <= switch_percent and next_speaker != last_question_id:
                        message_type = MessageType.SWITCH.value
                        question_id = next_speaker
                    else:
                        message_type = MessageType.OPEN.value
            elif last_message_directed != None and self.back_and_forth_counter < config.BACK_AND_FORTH_MAX and interrupt_check > config.INTERRUPT_PERCENT:
                # Last message was directed, and next one will be too
                next_speaker = last_message_directed # get the person who the last message was directed at
                message_type = MessageType.DIRECT.value
                directed_id = last_speaker # Respond to the most recent speaker
                if next_speaker == second_last_speaker:
                    self.back_and_forth_counter +=1
                else:
                    self.back_and_forth_counter = 0
            elif last_message_directed != None and (self.back_and_forth_counter >= config.BACK_AND_FORTH_MAX or interrupt_check <= config.INTERRUPT_PERCENT):
                # Interrupt a back and forth exchange 
                event_id = self.event_checker()
                if event_id != 0:
                    message_type = MessageType.EVENT.value
                else: 
                    message_type = MessageType.INTERRUPT.value
                # Choose anyone apart from last speaker and person before that
                filtered_members = [item for item in group_members if item != last_speaker and item != second_last_speaker]
                next_speaker = random.choice(filtered_members)
                self.back_and_forth_counter = 0
            elif last_message_directed == None: # last message was not directed at anyone
                # Choose anyone apart from last speaker AND second to last speaker (as more than 2 people in group)
                filtered_members = [item for item in group_members if item != last_speaker and item != second_last_speaker]
                next_speaker = random.choice(filtered_members)
                # Last message was not directed; next one doesn't need to be
                # But could be based on some percentage
                # Choose if the message should be directed at anyone
                # Should NOT be directed at someone random - should always be directed at who just spoke ! (otherwise it sounds weird)
                event_id = self.event_checker()
                if event_id != 0:
                    message_type = MessageType.EVENT.value
                else:
                    rand = random.randint(0, 100)
                    if rand < config.DIRECT_PERCENT:
                        message_type = MessageType.DIRECT.value
                        # Direct at last speaker
                        directed_id = last_speaker
                    elif rand >= config.DIRECT_PERCENT and rand < (config.DIRECT_PERCENT + switch_percent) and next_speaker != last_question_id:
                        message_type = MessageType.SWITCH.value
                        question_id = next_speaker
                    else: 
                        message_type = MessageType.OPEN.value
                self.back_and_forth_counter = 0
            else:
                print("ERROR! In unexpected part of if/else statement.")

        # Check if the question_id is for a person who is still in the group...
        # If not, then switch the question_id to current speaker
        if question_id not in group_members:
            message_type = MessageType.SWITCH.value
            question_id = next_speaker

        return next_speaker, message_type, directed_id, event_id, question_id, question_phase

            
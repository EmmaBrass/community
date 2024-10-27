import community.configuration as config
from community.message_type import MessageType

from ament_index_python.packages import get_package_share_directory

import random, time, os, yaml

class GroupConvoManager():
    """
    Manages who speaks next in a conversation within a group.
    """
    # TODO add hello and goodbye (joining and leaving) message types into this class
    # TODO global events are checked here!  And then sent to person as an instruction.  Check events.yaml

    def __init__(self):
        # Keep track of how long a convo has been back and forth between two people
        self.back_and_forth_counter = 0

        # Get the path to the `events.yaml` file
        package_share_dir = get_package_share_directory('community')
        events_path = os.path.join(package_share_dir, 'config_files', 'events.yaml')

        with open(events_path, 'r') as file:
            events_yaml = yaml.safe_load(file)
        self.events_data = events_yaml['events']

        # Note the time the group_node was created, for checking against events
        self.initialise_time = time.time()

        # Keep track of which events this group has already discussed
        self.discussed_event_ids = []

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
        time_now = time.time()
        elapsed_seconds = time_now - self.initialise_time
        # Check if the current time is past the event time
        for event_id, event in self.events_data.items():
            timestamp = event['timestamp']
            timestamp_seconds = self.convert_to_seconds(timestamp)
            if elapsed_seconds > timestamp_seconds and elapsed_seconds < (timestamp_seconds + config.MAX_EVENT_DISCUSS_WAIT):
                # Check if that event has already been discussed by this group
                if event_id not in self.discussed_event_ids:
                    self.discussed_event_ids.append(event_id)
                    return int(event_id)  # Return event_id as integer
        return 0  # Returns 0 if no event to talk about, otherwise return event_id to be discussed.


    def get_next(self, group_members, last_speaker, second_last_speaker, last_message_directed=0):
        """
        Get next speaker and speech type.

        :param group_members: Current group members.
        :param last_speaker: Who spoke most recently.
        :param last_message_directed: Who the last message was directed at, if anyone.

        :returns next_speaker: The next group member to talk
        :returns message_type: What type of thing they are going to say.
        :returns direct_to: The person the next message should be directed at (0 if none)
        :returns event_id: If of event to be talked about (0 if none)
        """

        # directed_id is 0 (noone) by default (message not directed at anyone)
        directed_id = 0
        # event_id is 0 by default (no event to speak about)
        event_id = 0

        if len(group_members) == 1:
            next_speaker = group_members[0]
            event_id = self.event_checker()
            if event_id != 0:
                message_type = MessageType.EVENT.value
            else: 
                message_type = MessageType.ALONE.value
            back_and_forth_counter = 0

        elif len(group_members) == 2:
            # Next speaker is person in group_members who is not last_speaker!
            filtered_members = [item for item in group_members if item != last_speaker]
            next_speaker = random.choice(filtered_members)
            filtered_members = [item for item in group_members if item != next_speaker]
            directed_id = filtered_members[0] # Drect next message to the other person
            event_id = self.event_checker()
            if event_id != 0:
                message_type = MessageType.EVENT.value
            else: 
                message_type = MessageType.DIRECT.value
            back_and_forth_counter = 0 # back and forth counter doesn't apply if only 2 people in the group

        elif len(group_members) > 2:
            interrupt_check = random.randint(0,100)
            if last_speaker == 0:
                # Noone has spoken yet - startup of the system
                # Just choose someone random from existing members
                next_speaker = random.choice(group_members)
                event_id = self.event_checker()
                if event_id != 0:
                    message_type = MessageType.EVENT.value
                else: 
                    message_type = MessageType.OPEN.value
            elif last_message_directed != 0 and back_and_forth_counter < config.BACK_AND_FORTH_MAX and interrupt_check > config.INTERRUPT_PERCENT:
                # Last message was directed, and next one will be too
                next_speaker = last_message_directed # get the person who the last message was directed at
                message_type = MessageType.DIRECT.value
                directed_id = last_speaker # Respond to the most recent speaker
                if next_speaker == second_last_speaker:
                    back_and_forth_counter +=1
                else:
                    back_and_forth_counter = 0
            elif last_message_directed != 0 and (back_and_forth_counter >= config.BACK_AND_FORTH_MAX or interrupt_check <= config.INTERRUPT_PERCENT):
                # Interrupt a back and forth exchange 
                event_id = self.event_checker()
                if event_id != 0:
                    message_type = MessageType.EVENT.value
                else: 
                    message_type = MessageType.INTERRUPT.value
                # Choose anyone apart from last speaker and person before that
                filtered_members = [item for item in group_members if item != last_speaker and item != second_last_speaker]
                next_speaker = random.choice(filtered_members)
                back_and_forth_counter = 0
            elif last_message_directed == 0:
                # Choose anyone apart from last speaker
                filtered_members = [item for item in group_members if item != last_speaker]
                next_speaker = random.choice(filtered_members)
                # Last message was not directed; next one doesn't need to be
                # But could be based on some percentage
                # Choose if the message should be directed at anyone
                event_id = self.event_checker()
                if event_id != 0:
                    message_type = MessageType.EVENT.value
                else:
                    rand = random.randint(0, 100)
                    if rand < config.DIRECT_PERCENT:
                        message_type = MessageType.DIRECT.value
                        # Direct at someone
                        filtered_members = [item for item in group_members if item != last_speaker and item != next_speaker]
                        directed_id = random.choice(filtered_members)
                    else: 
                        message_type = MessageType.OPEN.value
                back_and_forth_counter = 0
            else:
                print("ERROR! In unexpected part of if/else statement.")
                
        return next_speaker, message_type, directed_id, event_id

            
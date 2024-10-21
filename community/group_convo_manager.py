import community.configuration as config
from enum import Enum

import random

class MessageType(Enum):
    JOINING = 0 # Say hello because just joined a group
    LEAVING = 1 # Say bye because leaving a group (likely won't be used...)
    OPEN = 2 # Say whatever into the group
    ALONE = 3 # A person alone in a group
    INTERRUPT = 4 # A new person interrupting a back-and-forth
    DIRECT = 5 # A direct message aimed at someone in particular
    EVENT = 6 # Talk about some global event that has just happened.

class GroupConvoManager():
    """
    Manages who speaks next in a conversation within a group.
    """
    # TODO add hello and goodbye (joining and leaving) message types into this class
    # TODO global events are checked here!  And then sent to person as an instruction.  Check events.yaml

    def __init__(self):
        # Keep track of how long a convo has been back and forth between two people
        self.back_and_forth_counter = 0
        # Keep track of the order of who has spoken
        speakers = []

    def get_next(self, group_members, last_speaker, last_message_directed=0):
        """
        Get next speaker and speech type.

        :param group_members: Current group members.
        :param last_speaker: Who spoke most recently.
        :param last_message_directed: Who the last message was directed at, if anyone.

        :returns next_speaker: the next group member to talk
        :returns message_type: what type of thing they are going to say.
        :param direct_to: the person the next message should be directed at
        """

        if last_speaker != 0:
            self.speakers.append(last_speaker)

        # directed_to is 0 (noone) unless changed later
        directed_id = 0

        if len(group_members) == 1:
            next_speaker = group_members[0]
            message_type = MessageType.ALONE.value
            back_and_forth_counter = 0

        elif len(group_members) == 2:
            # Next speaker is person in group_members who is not last_speaker!
            filtered_members = [item for item in group_members if item != last_speaker]
            next_speaker = random.choice(filtered_members)
            filtered_members = [item for item in group_members if item != next_speaker]
            directed_id = filtered_members[0] # Drect next message to the other person
            message_type = MessageType.DIRECT.value
            back_and_forth_counter = 0 # back and forth counter doesn't apply if only 2 people in the group

        elif len(group_members) > 2:
            interrupt_check = random.randint(0,100)
            if last_speaker == 0:
                # Noone has spoken yet - startup of the system
                # Just choose someone random from existing members
                next_speaker = random.choice(group_members)
            elif last_message_directed != 0 and back_and_forth_counter < config.BACK_AND_FORTH_MAX and interrupt_check > config.INTERRUPT_PERCENT:
                # Last message was directed, and next one will be too
                next_speaker = last_message_directed # get the person who the last message was directed at
                message_type = MessageType.DIRECT.value
                directed_id = last_speaker # Resoond to the most recent speaker
                if next_speaker == self.speakers[-2]:
                    back_and_forth_counter +=1
                else:
                    back_and_forth_counter = 0
            elif last_message_directed != 0 and (back_and_forth_counter >= config.BACK_AND_FORTH_MAX or interrupt_check <= config.INTERRUPT_PERCENT):
                # Interrupt a back and forth exchange 
                message_type = MessageType.INTERRUPT.value
                # Choose anyone apart from last speaker and person before that
                filtered_members = [item for item in group_members if item != last_speaker and item != self.speakers[-2]]
                next_speaker = random.choice(filtered_members)
                back_and_forth_counter = 0
            elif last_message_directed == 0:
                # Choose anyone apart from last speaker
                filtered_members = [item for item in group_members if item != last_speaker]
                next_speaker = random.choice(filtered_members)
                # Last message was not directed; next one doesn't need to be
                # But could be based on some percentage
                # Choose if the message should be directed at anyone
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
                
        return next_speaker, message_type, directed_id

            
from enum import Enum

class MessageType(Enum):
    JOINING = 0 # Say hello because just joined a group
    LEAVING = 1 # Say bye because leaving a group (likely won't be used...)
    OPEN = 2 # Say whatever into the group about the current question
    ALONE = 3 # You are a person alone in a group
    INTERRUPT = 4 # You are interrupting a back-and-forth
    DIRECT = 5 # A direct message aimed at someone in particular
    EVENT = 6 # Talk about some global event that has just happened.
    SWITCH = 7 # Switch the conversation to now be about your own personal question. 

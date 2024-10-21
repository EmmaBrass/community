from enum import Enum

class MessageType(Enum):
    JOINING = 0 # Say hello because just joined a group
    LEAVING = 1 # Say bye because leaving a group (likely won't be used...)
    OPEN = 2 # Say whatever into the group
    ALONE = 3 # A person alone in a group
    INTERRUPT = 4 # A new person interrupting a back-and-forth
    DIRECT = 5 # A direct message aimed at someone in particular
    EVENT = 6 # Talk about some global event that has just happened.

class PromptManager():
    """
    Takes output from relationship manager (state_changed bool, from_state, to_state, action)
    Message type as required by group convo manager.
    Craft the prompt specificiations from this information.
    """
    
    def __init__(self):
        pass

    def get_relationship_prompt(sel, message_type: int, directed_id: int, state_changed: bool, from_state: str, to_state: str, action: str):
        """
        This person has been asked to speak.
        Looks at requested message type.
        Looks at if the speech should be directed at anyone.
        Take stock of existing relationships.
        Check for relationship CHANGES since the last time this person spoke.  THESE will be worth commenting on.
        Check if there is a command to mention an event from the event timeline (should be passed an event id in the ROS msg).
        Craft the prompt specificiations from this information.

        :returns prompt_details: A string to be passed to the LLM to help guide its output text.
        """
        # Maybe a prompt 
        
        if message_type == 0:
            # respond to previous thing and say hello
        
        elif message_type == 1:
            # respond to previous thing and say bye?

        elif message_type == 2:
            # just respond to previous thing for 2 sentences

        elif message_type == 3:
            # talk baout feeling alone

        elif message_type == 4: 
            # interrupt previous back and forth; comment on what has been said rather than introducing a new topic.

        elif message_type == 5:
            # get name of person the message is directed at using directed_id
            # check if state of the relationship has changed and comment on that
            # check if any action to discuss and comment on that if so

        elif message_type == 6:
            # use event_id to get event description and urgency and discuss it.  



    # Need like set formats for relationship types - maybe classes for these?  and state machines for these that get ticked through.  
    # A TICK = an interaction !!! So for each interaction that this class sees, we will ask the state machine for that relationship to tick...
    # hmmm no but state machine needs to be shared somehow... accessible by the people managers of both classes.  

    # given the speech type from the groupconvomanager, 
    # decide what to say
    # update relationships from here
    # Decide HOW a relationship is going to change, potentially based on num interactions in past.
    # That change happens kind of DURING this turn and directly affect what insturctions we give the LLM.

    #e.g. CHECK on all relationships with other gorup members and where they are at, check what action require dnext with any of them.
    # check relationship.txt files, make some decision what to do based on the states of all those, update the .txt file, tell the LLM to comment on
    # this action as appropriate.  
    # Here is the engine for a person to make a decision what to sa next based on what was just said to them and what they want to do netx in their relationships.

    # options: 
    # Simple comment on existing topic - let the LLM say someting generic

    # Comment on existing topic wih some input from own personality and interests

    # A relationship update with the person who just spoke, comment is based on that.
        # Maybe speeches can also have 'TYPES' - like a job offer, a recruitment drive, relationship proposal, friendship proposal, relationship break-up etc.
        # I think yes to this.  This is passed through, followed by LLM, but also kept as a rigid data for a person to update their relationship txt files and 
        # then decide how to respond...
    #Hmmm or no, could also do a SEARCH for relationship updates with others.  e.g. the person who just spoke might also have just updated
    # their relationship, and that needs to be taken into account... 
    # Relationships could have like a 'last move' flag?  So the last person to make a directed comment between them is saved
    # and then with this comment type, would only be done with relationships where it is this person's turn.

    # Comment based on external event from event config file.



    # messages should also be sent out with a reponse_priority variable...
    # would be weird if a big event happened and then a person makes one comment on it and then the rest of the group just ignores
    # and moves on to something else.


    # This will be queried when someone else speaks to the person, AND right before the person is asked to speak.
    # Is a kind of model of what mood that person is in.
    # Also will access and update txt files with the information about the RELATIONSHIP this person has with all other people.

    def check_relationships(self):
        """
        Check all existing relationships and flag those where it is 
        this person's turn to speak.
        """

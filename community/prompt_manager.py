from community.message_type import MessageType
import yaml, os, random
from ament_index_python.packages import get_package_share_directory

class PromptManager():
    """
    Takes output from relationship manager (state_changed bool, from_state, to_state, action)
    Message type as required by group convo manager.
    Craft the prompt specificiations from this information.
    """
    
    def __init__(self):

        self.interrupt_responses = {
            0 : "You are bored of hearing them speak, and tell them so.",
            1 : "You are really excited about their conversation topic and you give them your own opinion.",
            2 : "You know more about what they are talking about than they do.  You offer them your specific insights.",
            3 : "You are very curious about their conversation topic and you ask a pertinent question."
        }

        self.alone_responses = {
            0 : "You say you are happy to be alone.  People overwhelm you sometimes.",
            1 : "You comment on how you feel lonely.",
            2 : "You say you might go completely crazy here alone, because humans are social animals after all.",
            3 : "You mumble gibberish to yourself, like you are thinking aloud."
        }

        self.open_responses = {
            0 : "You respond to whatever topic people have been discussing most recently in the group.",
            1 : "You say one sentence in response to the most recent conversation topic, \
                then another sentence to move to an adjacent topic.",
            2 : "You respond to the current conversation topic, expressing excitement for this topic.",
            3 : "You respond to the current conversation topic, but also expressing exasperation for this topic, \
                with a reason for your exasperation."
        }

        self.event_urgency_dict = {
            1 : {
                'range' : [0,25],
                'description' : "This is not very important news, don't make a big deal out of it."
            },
            2 : {
                'range' : [25,50],
                'description' : "This is kind-of important news, people will want to know.",
            },
            3 : {
                'range' : [50,75],
                'description' : "This is big news.  Everyone needs to hear this.",
            },
            4 : {
                'range' : [75,100],
                'description' : "This is world-changing news.  It is IMPERATIVE that everyone listens."
            }
        }


        # Get the path to the 'events.yaml' and the 'people.yml' file
        package_share_dir = get_package_share_directory('community')
        events_path = os.path.join(package_share_dir, 'config_files', 'events.yaml')
        people_path = os.path.join(package_share_dir, 'config_files', 'people.yaml')

        self.people_data = self.load_people(people_path)
        self.events_data = self.load_events(events_path)

    def load_events(self, file_path):
        """ Load events data from the YAML file. """
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data['events']

    def get_event_description_by_id(self, event_id):
        """ Function to get an event's description by ID. """
        event = self.events_data.get(event_id)  # Ensure ID is treated as a string
        if event:
            return event.get('description')
        else:
            raise LookupError("event not found!")
    
    def get_event_urgency_by_id(self, event_id):
        """ Function to get an event's urgency by ID. """
        event = self.events_data.get(event_id)  # Ensure ID is treated as a string
        if event:
            return event.get('urgency')
        else:
            raise LookupError("event not found!")

    def load_people(self, file_path):
        """ Load people data from the YAML file. """
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data['people']
    
    def get_name_by_id(self, person_id):
        """ Function to get a person's name by ID. """
        person = self.people_data.get(person_id)  # Ensure ID is a string for key lookup
        if person:
            return person.get('name')
        else:
            raise LookupError("person not found!")

    def get_prompt_details(self, message_type: int, directed_id: int, event_id: int, state_changed: bool, from_state: str, to_state: str, action: str):
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
        
        if MessageType(message_type).name == 'JOINING':
            # Say hello
            prompt_details = "You have just joined the group.  Say hello in a few words."
        elif MessageType(message_type).name == 'LEAVING':
            # respond to previous thing and say bye.
            prompt_details = "You are leaving the group.  Respond breifly to the current \
                conversation topic and then say goodbye in a few words."
        elif MessageType(message_type).name == 'OPEN':
            # just respond to previous thing for 2 sentences
            prompt_details = self.open_responses[random.randint(0,3)]
        elif MessageType(message_type).name == 'ALONE':
            # talk baout feeling alone
            prompt_details = f"You are the only one in the group. {self.alone_responses[random.randint(0,3)]}."
        elif MessageType(message_type).name == 'INTERRUPT': 
            # interrupt previous back and forth; comment on what has been said rather than introducing a new topic.
            prompt_details = f"You are interrupting a back-and-forth between two people. {self.interrupt_responses[random.randint(0,3)]}."
        elif MessageType(message_type).name == 'DIRECT':
            prompt_details = ""
            # Get name of person the message is directed at using directed_id
            directed_name = self.get_name_by_id(directed_id)
            prompt_details += f"This response will be directed at {directed_name}."
            # Check if state of the relationship has changed and comment on that
            if state_changed == True:
                prompt_details += f"The state of your relationship with this person has just changed from {from_state} to {to_state}.  Comment on this!"
            if action != 'None':
                prompt_details += f"Say to the other person something like: {action}"
        elif MessageType(message_type).name == 'EVENT':
            # Use event_id to get event description and urgency and discuss it.  
            event_description = self.get_event_description_by_id(event_id)
            prompt_details = f"You just heard some news! {event_description} Talk about this."
            event_urgency = self.get_event_urgency_by_id(event_id)
            event_urgency_description = None
            for _, value in self.event_urgency_dict:
                urgency_range = value['range']
                if event_urgency > urgency_range[0] and event_urgency <= urgency_range[1]:
                    event_urgency_description = value['description']
                    break
            if event_urgency_description != None:
                prompt_details += event_urgency_description
            else:
                print("Error! event_urgency_description not found.")

        return prompt_details



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

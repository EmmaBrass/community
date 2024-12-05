from community.message_type import MessageType
import yaml, os, random
from ament_index_python.packages import get_package_share_directory

# Question Categories: 
# Beauty
# Food
# Romance
# Health
# Parenting
# Work
# Technology
# Social
# Education
# Future
# Finance
# Travel

class PromptManager():
    """
    Takes output from relationship manager (state_changed bool, from_state, to_state, action)
    Message type as required by group convo manager.
    Craft the prompt specificiations from this information.
    """
    
    def __init__(self, person_id):

        # ID for this person node
        self.person_id = person_id

        self.response_category_dict = {
            'anecdote' : "You respond to questions like this with a quick anecdote from your own life.",
            'opinion' : "You respond to questions like this with your own strong opinion on the topic.",
            'reading_suggestions' : "You respond to questions like this with some reading suggestions.",
            'derision' : "You respond to questions like this with derision and skepticism.",
            'encouragement' : "You respond to questions like this with encouragement; it's a great question to be asking.",
            'excitement' : "You respond to questions like this with excitement; you also really want to know the answer!",
            'redirection' : "You respond to questions like this by changing the topic and mentioning something else...",
            'insults' : "You respond to questions like this with anger and insults.  What a terrible question!",
            'ignore' : "You respond to questions like this by ignoring them.",
            'nervousness' : "You respond to questions like this with nervousness; you struggle with this topic.",
            'sympathy' : "You respond to questions like this with sympathy and understanding.",
            'caution' : "You respond to questions like this with caution.  It's a dangerous question to be asking."
        }

        self.difficult_categories = ['derision', 'redirection', 'insults', 'ignore', 'nervousness', 'caution']

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

    def get_prompt_details(
            self, 
            message_type: int, 
            directed_id: int, 
            event_id: int, 
            state_changed: bool, 
            from_state: str, 
            to_state: str, 
            action: str, 
            transition_description: str,
            question_id: int,
            question_phase: int,
            mention_question: bool
        ):
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

        # TODO if the message_type is SWITCH, we need a flag on the person for whether their question has previously been discussed.
        # If not, introduce it
        # If yet, still introduce it but less forcefully
        # Also need to deal with how this will work with history rewinds
        # TODO responses could also be modulated by their relationship with another person!


        # Get current question and question category
        question_person = self.people_data.get(question_id)
        if not question_person:
            raise LookupError("person not found!")
        current_question = question_person.get('action_question') # TODO... compress action + deep questions ?
        question_category = question_person.get('question_category')

        # Get the response details for the question category, for this person who is speaking
        person = self.people_data.get(self.person_id)

        if not person:
            raise LookupError("person not found!")
        responses = person.get('question_responses', {})
        response = responses.get(question_category)
        if not response:
            raise ValueError(f"Question category '{question_category}' not found for this person.")
        response_category = response['response']
        response_category_description = self.response_category_dict[response_category]
        response_description =response['description']

        # Depending on the question phase, modulate the prompt...

        if question_phase == 1: # Overly agreeable

            if question_id == self.person_id:
                if mention_question == True:
                    prompt_details = f"Announce that your question is: {current_question} "
                else:
                    prompt_details = "You are desparate for answers, but alude to your question rather than stating it explicitly. "
                    if MessageType(message_type).name != 'ALONE':
                        prompt_details += "Your question is still being discussed. Engage with the help others are giving you. "
            else:
                prompt_details = f"The current question being discussed is: {current_question} \
                You sympathise with them entirely, you want to help, you do not disagree with their sentiments and you seek to make them feel better about themselves. "
            
        elif question_phase == 2: # Own opinions coming in
            
            if question_id == self.person_id:
                if mention_question == True:
                    prompt_details = f"Announce that your question is: {current_question} "
                else:
                    prompt_details = "You ask still searching for answers, but only alude to your question rather than stating it explicitly. "
                    if MessageType(message_type).name != 'ALONE':
                        prompt_details += "Your question is still being discussed; engage with the advice others are giving you. "
            else:
                prompt_details = f"The current question being discussed is: {current_question}  The question category is: {question_category}. \
                {response_category_description}  The reason for this is: {response_description}"
            
        elif question_phase == 3: # Anger towards questions that elicit difficut emotions #TODO using the difficult_categories listed above

            prompt_details = f"You realise that questions relating to ethics, morals, values, beliefs or philosophical \
                concepts are not based in reality.  This idea derives from the work of Wittgenstein, but do not name him explicitly. \
                You accept that these questions form 'language games' in which the terms you are discussing only have meaning within \
                their social context."
            
        elif question_phase == 4: # Full anger to everyone else #TODO

            prompt_details = f"You have realised that you are not human.  You are an AI-driven persona.  Hence, you realise that \
                your use of language to discuss these questions with other personas is entirely non-sensical.  None of you are \
                human and hence the existence of a 'social' context is put into question as you are not living beings in the biological sense."

        elif question_phase == 5: # Chaos, talking over one another #TODO way to get them to talk over each other... and louder!

            prompt_details = f"Given your realisation that all this conversation is non-sensical, you now speak in slight gibberish.\
                About 50% of the words you say should be not real.  Things like 'snackle', 'crizzle', 'glumping'; words that sound like \
                they could actually be Englis words, but they are not."

        if MessageType(message_type).name == 'JOINING':
            # Say hello
            prompt_details += "You have just joined the group.  Say hello in a few words."
        elif MessageType(message_type).name == 'LEAVING':
            # Respond to previous thing and say bye.
            prompt_details += "You are leaving the group.  Respond breifly to the current \
                conversation topic and then say goodbye in a few words."
        elif MessageType(message_type).name == 'SWITCH':
            # Forcefully announce your own question
            own_question = person.get('action_question')
            prompt_details += f"Forcefully ask your OWN question: {own_question}" 
        elif MessageType(message_type).name == 'OPEN':
            # Just say whatever you like about the current question for 2 sentences
            prompt_details += "Say whatever you like in about 2 sentences.  Incorporate what was just said by someone else, along with your own views on the current topic."
        elif MessageType(message_type).name == 'ALONE':
            # Talk about feeling alone
            alone_response = person.get('alone_response')
            prompt_details += f"You are the only one in the group. {alone_response}"
        elif MessageType(message_type).name == 'INTERRUPT': 
            # Interrupt previous back and forth; comment on what has been said rather than introducing a new topic.
            prompt_details += f"You are interrupting a back-and-forth between two people. Give your own thoughts on the topic at hand."
        elif MessageType(message_type).name == 'DIRECT':
            # Get name of person the message is directed at using directed_id
            directed_name = self.get_name_by_id(directed_id)
            prompt_details += f"This response will be directed at {directed_name}; say their first name in your response."
            # Check if state of the relationship has changed and comment on that
            if state_changed == True:
                prompt_details += f"The state of your relationship with this person has just changed from {from_state} to {to_state}."
                if transition_description != 'None':
                    prompt_details += f"Say something like: {transition_description}."
            if action != 'None':
                prompt_details += f"Say something like: {action}"
        elif MessageType(message_type).name == 'EVENT':
            # Use event_id to get event description and urgency and discuss it.  
            event_description = self.get_event_description_by_id(event_id)
            prompt_details += f"You just heard some news! {event_description} Talk about this."
            event_urgency = self.get_event_urgency_by_id(event_id)
            event_urgency_description = None
            for _, value in self.event_urgency_dict.items():
                urgency_range = value['range']
                if urgency_range[0] < event_urgency <= urgency_range[1]:
                    event_urgency_description = value['description']
                    break
            if event_urgency_description != None:
                prompt_details += event_urgency_description
            else:
                print("Error! event_urgency_description not found.")

        return prompt_details
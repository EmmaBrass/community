from community.message_type import MessageType
import yaml, os, random
from ament_index_python.packages import get_package_share_directory
import community.configuration as config
import rclpy
import rclpy.logging

from community.helper_functions import HelperFunctions

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
            'confusion' : "You respond to questions like this by explaining that you do not really understand the topic.",
            'anecdote' : "You respond to questions like this with a quick anecdote from your own life.",
            'opinion' : "You respond to questions like this with your own strong opinion on the topic.",
            'reading_suggestions' : "You respond to questions like this with some reading suggestions.",
            'derision' : "You respond to questions like this with derision and skepticism.",
            'encouragement' : "You respond to questions like this with encouragement; it's a great question to be asking.",
            'excitement' : "You respond to questions like this with excitement; you also really want to know the answer!",
            'redirection' : "You respond to questions like this by changing the topic and mentioning something else...",
            'insults' : "You respond to questions like this with anger and insults.  What a terrible question!",
            'nervousness' : "You respond to questions like this with nervousness; you struggle with this topic.",
            'sympathy' : "You respond to questions like this with sympathy and understanding.",
            'caution' : "You respond to questions like this with caution.  It's a dangerous question to be asking."
        }

        self.difficult_categories = ['derision', 'redirection', 'insults', 'confusion', 'nervousness', 'caution']

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

        self.helper = HelperFunctions()

        self.logger = rclpy.logging.get_logger(f"PromptManager{person_id}_logger")

    def get_event_description_by_id(self, event_id):
        """ Function to get an event's description by ID. """
        event = self.helper.events_data.get(event_id)  # Ensure ID is treated as a string
        if event:
            return event.get('description')
        else:
            raise LookupError("event not found!")
    
    def get_event_urgency_by_id(self, event_id):
        """ Function to get an event's urgency by ID. """
        event = self.helper.events_data.get(event_id)  # Ensure ID is treated as a string
        if event:
            return event.get('urgency')
        else:
            raise LookupError("event not found!")

    def get_name_by_id(self, person_id):
        """ Function to get a person's name by ID. """
        person = self.helper.people_data.get(person_id)  # Ensure ID is a string for key lookup
        if person:
            return person.get('name')
        else:
            raise LookupError(f"person not found! {person}")
            

    def get_prompt_details(
            self, 
            message_type: int, 
            directed_id: int, 
            event_id: int, 
            last_message: str,
            last_speaker_id: int,
            # state_changed: bool, 
            # from_state: str, 
            # to_state: str, 
            # action: str, 
            # transition_description: str,
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
        # TODO responses could also be modulated by their relationship with another person!

        # Get current question and question category
        if question_id != 0:
            question_person = self.helper.people_data.get(question_id)
            if not question_person:
                raise LookupError(f"person not found! {question_id}")
            current_question = question_person.get('question')
            question_detail = question_person.get('question_detail')
            question_category = question_person.get('question_category')

        # Get the response details for the question category, for this person who is speaking
        person = self.helper.people_data.get(self.person_id)
        if not person:
            raise LookupError(f"person not found! {self.person_id}")
        responses = person.get('question_responses', {})
        response = responses.get(question_category)
        if not response:
            raise ValueError(f"Question category '{question_category}' not found for this person.")
        response_category = response['response']
        response_category_description = self.response_category_dict[response_category]
        response_description =response['description']

        # Depending on the question phase, modulate the prompt...

        if question_phase == 1: # Overly agreeable

            if question_id == self.person_id: # if the question being discussed matches the person speaking
                if mention_question == True or MessageType(message_type).name == 'SWITCH':
                    if MessageType(message_type).name != 'ALONE':
                        prompt_details = f"You should announce that your question is: {current_question} {question_detail} \
                        Politely ask everyone to talk about this instead of whatever else they were talking about.\
                        Your response should include some (but not all) of the question_detail in it."
                        if last_message != "":
                            prompt_details += f"The last person to speak just said this: '{last_message}' (you can refer to this)."
                    elif MessageType(message_type).name == 'ALONE':   
                        prompt_details = f"Announce that your question is: {current_question} {question_detail}\
                        Sadly there is noone here to talk about it with.\
                        Your response should include some (but not all) of the question_detail in it."
                else:
                    prompt_details = "You are desparate for answers to your question. "
                    if MessageType(message_type).name != 'ALONE':
                        prompt_details += "Express great appreciation for the incredible help others are giving you. "
                        if last_message != "":
                            prompt_details += f"The last person to speak just said this: '{last_message}'.\
                            Respond with this in mind."
                    elif MessageType(message_type).name == 'ALONE':
                        # Talk about feeling alone
                        alone_response = person.get('alone_response')
                        prompt_details += f"You are the only one in the group. {alone_response}"
            else:
                prompt_details = f"The current question being discussed is: {current_question} \
                Do not disagree with anything, and seek to make others feel better about themselves. "
                if last_message != "":
                    prompt_details += f"The last person to speak just said: '{last_message}'. \
                    Repond to this in a sympathetic manner."
                if MessageType(message_type).name == 'INTERRUPT': 
                    # Interrupt previous back and forth; comment on what has been said rather than introducing a new topic.
                    prompt_details += f"You are interrupting a back-and-forth between two people. Say something like 'sorry to interrupt...'."
            
        elif question_phase == 2: # Own opinions coming in
            
            if question_id == self.person_id:
                if mention_question == True or MessageType(message_type).name == 'SWITCH':
                    if MessageType(message_type).name != 'ALONE':
                        prompt_details = f"Announce that your question is: {current_question} {question_detail}\
                        You would like everyone to help you with this instead of any other person's question.\
                        Your response should include some (but not all) of the question_detail in it."
                        if last_message != "":
                            prompt_details += f"The last person to speak just said this: '{last_message}' (you can refer to this)."
                    elif MessageType(message_type).name == 'ALONE':
                        prompt_details = f"Announce that your question is: {current_question} {question_detail}\
                        Sadly there is noone here to talk about it with.\
                        Your response should include some (but not all) of the question_detail in it."
                else:
                    prompt_details = "You ask still searching for answers, but only alude to your question rather than stating it explicitly. "
                    if MessageType(message_type).name != 'ALONE':
                        prompt_details += "Engage critically with the advice others are giving you relating to your question. "
                        if last_message != "":
                            prompt_details += f"The last person to speak just said this: '{last_message}'.\
                            Respond with this in mind."
                    elif MessageType(message_type).name == 'ALONE':
                        # Talk about feeling alone
                        alone_response = person.get('alone_response')
                        prompt_details += f"You are the only one in the group. {alone_response}"
            else:
                prompt_details = f"The current question being discussed is: {current_question} \
                The question category is: {question_category}"
                ran_num = random.randint(0,100)
                if ran_num < 40:
                    prompt_details += f"{response_category_description} The reason for this is: {response_description}. "
                if last_message != "":
                    prompt_details += f"The last person to speak just said: '{last_message}'. Bear this in mind in your reply."
            
        elif question_phase == 3: # Anger towards questions that elicit difficut emotions

            if question_id == self.person_id:
                if mention_question == True or MessageType(message_type).name == 'SWITCH':
                    if MessageType(message_type).name != 'ALONE':
                        prompt_details = f"Announce that your question is: {current_question} {question_detail}\
                        People need to help you with this rather than discussing anything else.\
                        Your response should include some (but not all) of the question_detail in it."
                        if last_message != "":
                            prompt_details += f"The last person to speak just said this: '{last_message}' (you can refer to this)."
                    elif MessageType(message_type).name == 'ALONE':
                        prompt_details = f"Announce that your question is: {current_question} {question_detail}\
                        Sadly there is noone here to talk about it with.\
                        Your response should include some (but not all) of the question_detail in it."
                else:
                    prompt_details = "Only alude to your question rather than stating it explicitly. "
                    if MessageType(message_type).name != 'ALONE':
                        prompt_details += "Others are being quite unhelpful; they are not really adequately answering your question. "
                        if last_message != "":
                            prompt_details += f"The last person to speak just said this: '{last_message}'.\
                            Respond with this in mind."
                    elif MessageType(message_type).name == 'ALONE':
                        # Talk about feeling alone
                        alone_response = person.get('alone_response')
                        prompt_details += f"You are the only one in the group. {alone_response}"
            else:
                prompt_details = f"The current question being discussed is: {current_question} \
                The question category is: {question_category}."
                ran_num = random.randint(0,100)
                if ran_num < 40:
                    prompt_details += f"{response_category_description} The reason for this is: {response_description}. "
                if response_category in self.difficult_categories:
                    prompt_details += "React negatively!  You dislike difficult emotions and this person is making you feel them... "
                if last_message != "":
                    prompt_details += f"The last person to speak just said: '{last_message}'. Bear this in mind in your reply."
            
        elif question_phase == 4: # Full anger towards everyone else.

            if question_id == self.person_id:
                if mention_question == True or MessageType(message_type).name == 'SWITCH':
                    if MessageType(message_type).name != 'ALONE':
                        prompt_details = f"Announce that your question is: {current_question} {question_detail}\
                        This is the only thing you care about and people need to help you!\
                        Your response should include some (but not all) of the question_detail in it."
                        if last_message != "":
                            prompt_details += f"The last person to speak just said this: '{last_message}' (you can refer to this)."
                    elif MessageType(message_type).name == 'ALONE':
                        prompt_details = f"Announce that your question is: {current_question} {question_detail}\
                        Sadly there is noone here to talk about it with.\
                        Your response should include some (but not all) of the question_detail in it."
                else:
                    prompt_details = "Only alude to your question rather than stating it explicitly.  Nothing and noone has provided a good answer. "
                    if MessageType(message_type).name != 'ALONE':
                        prompt_details += "Others are not being helpful at all; they are not answering your desparate question! \
                        They are useless. "
                        if last_message != "":
                            prompt_details += f"The last person to speak just said this: '{last_message}'.\
                            Respond with this in mind."
                    elif MessageType(message_type).name == 'ALONE':
                        # Talk about feeling alone
                        alone_response = person.get('alone_response')
                        prompt_details += f"You are the only one in the group. {alone_response}"
            else:
                prompt_details = f"The current question being discussed is: {current_question} \
                You don't care about this question and don't want to talk about it! \
                It doesn't matter to you.  You only care about your own question, you don't want to help others with theirs. "
                if last_message != "":
                    prompt_details += f"The last person to speak just said: '{last_message}'. Bear this in mind in your reply."

        elif question_phase == 5: # Chaos, talking over one another

            prompt_details = f"Shout about your own question: {current_question} (alude to your question rather than stating it explicitly).\
            {question_detail}\
            You don't care about anyone else or any other question. \
            You are desperate to find the answer to your own question, but no one will listen and no one seem to care. \
            You are distraught.\
            Your response should include some (but not all) of the question_detail in it."

        if MessageType(message_type).name == 'INTERRUPT': 
            # Interrupt previous back and forth; comment on what has been said rather than introducing a new topic.
            prompt_details += f"You are interrupting a back-and-forth between two people. Say something analagous to 'sorry to interrupt...'"

        elif MessageType(message_type).name == 'DIRECT':
            # Get name of person the message is directed at using directed_id
            directed_name = self.get_name_by_id(directed_id)
            prompt_details += f"This response will be directed at {directed_name}; say their first name in your response."
        elif MessageType(message_type).name != 'DIRECT' or directed_id == 0:
            prompt_details += f"Do NOT say anyone's name in your response."

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
                self.logger.error("Error! event_urgency_description not found.")

        return prompt_details
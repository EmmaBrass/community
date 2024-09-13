
from openai import OpenAI
import json
import os
import re
import time
from datetime import datetime
import community.configuration as config

# interactions as a dict... a list of all the text they have exchanged with other people in order, with time stamps?
# or a list of dicts perhaps.  [{date_time: ''}, {group: ''}, {name: ''}, {text: ''}]
# - group 3 - 
# 2024-05-10-16-44-45 : group 3 : Jessica Rust : Hey how are you?
# 2024-05-10-16-45-17 : group 3 : Self : Fine thanks, how are you?

class Person():

    def __init__(
        self, 
        person_id: int,
        name: str, 
        gender: str,
        age: int, 
        openness: int, 
        conscientiousness: int, 
        neuroticism: int, 
        agreeableness: int, 
        extraversion: int,
        history: str,
        relationships: dict
    ):
        self.name = name
        self.age = age
        self.gender = gender
        self.big_five_traits = {
            'openness': openness, 
            'conscientiousness': conscientiousness,
            'neuroticism': neuroticism,
            'agreeableness': agreeableness,
            'extraversion': extraversion
        }
        self.history = history # Where they come from, their job, what they like and dislike.
        self.relationships = relationships # TODO relationships should be UPDATED when a person leaves a group, based on the convo that just happened.
        # This means new relationships can be formed if a new person was in the group.  Maybe also a .txt file ?
        self.interactions = [] # create interactions list
        self.initialise_gpt()
        self.datetime_format = "%Y-%m-%d_%H-%M-%S"

    def initialise_gpt(self):
        """
        Create the GPT instance and feed it the intro_instructions and
        initial data and history for the person.
        """
        intro_instructions = f"This is an art installation.  You embody a human,\
            with a full life and backstory.  You will assigned a group, in which you will\
            be placed with 1 to 5 other people.  You will all have a conversation.\
            When you are placed in a group, you will be reminded of all the past interations\
            you have had with the others in this group.  When a user message command is given, you should\
            only response with NONE, unless the command contains 'RESPOND', i.e. only give a real response\
            that is not NONE if the command is <RESPOND-JOINING>, <RESPOND-LEAVING>, or <RESPOND-NORMAL>;\
            this is very important.\n\
            You will be given one of these commands:\n\
            <NEW GROUP> You have joined a new group and are given a summary of your past \
            interactions with each of the group members.\n\
            <TEXT> Another member of your group is speaking. You will be given their name, and \
            the text for what they say.\n\
            <MEMBER LEFT> You will be given the name of the person who has left the group.\n\
            <MEMBER JOINED> You will be given the name of the person who has joined the group.\n\
            <RESPOND-NORMAL> It is your turn to speak.  Say something that continues the conversation.\n\
            This command may include some instructions on what topic you should speak about.\n\
            <RESPOND-JOINING> Say hello to everyone in your new group.\n\
            <RESPOND-LEAVING> Say goodbye to everyone in the group you are leaving.\n\
            Before we start, I will give you some information about yourself.  \
            Your name is {self.name} and you are a {self.gender} and {self.age} years old.  \
            To describe your personality, I will give you a score from 0 (low) \
            to 10 (high) on each of the Big Five personality traits:\n\
            openness: {self.big_five_traits['openness']}\n\
            conscientiousness: {self.big_five_traits['conscientiousness']}\n\
            neuroticism: {self.big_five_traits['neuroticism']}\n\
            agreeableness: {self.big_five_traits['agreeableness']}\n\
            extraversion: {self.big_five_traits['extraversion']}\n\
            Here is a brief description of your life and imporant things that \
            have happened to you: {self.history}.\n\
            And here is a description of your relationships with people who you \
            already know: {self.relationships}\n\
            These relationships are important.  You should let your pre-existing relationship \
            with a person guide how you interact with them if you are ever in the same group \
            conversation." #TODO try with and without reminder of past interactions w/ group members
            # TODO make these (apart from <RESPOND>) into function calls???
        self.api_key = "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"
        self.client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY", self.api_key))
        self.create_assistant(intro_instructions)
        self.create_thread()

    def person_speaks(
        self, 
        person_id: int, 
        group_id: int, 
        people_in_group: list, # Members of the group EXCLUDING this person.
        message_type: int
    ):
        """
        Request text from THIS PERSON.

        :param person_id: The ID for this person
        :param group_id: The ID for the group this person is in
        :param people_in_group: The other people in the group, 
        EXCLUDING this person.
        :param message_type: The type of message you want; 0=joining, 1=leaving, 2=normal convo.
        returns text: The text response from the GPT.
        """
        if message_type == 0:
            text = self.add_user_message_and_get_response("<RESPOND-JOINING>")
        elif message_type == 1:
            text = self.add_user_message_and_get_response("<RESPOND-LEAVING>")
        elif message_type == 2:
            text = self.add_user_message_and_get_response("<RESPOND-NORMAL>")
        # Save the text to the interactions memory dict
        self.update_interactions_dict(person_id, group_id, people_in_group, text)
        return text

    def member_joined_group(self, person_id: int):
        """
        Inform the GPT that someone has joined the group.
        
        :param person_id: The ID for the person who has joined the group.
        :returns response: Should be NONE
        """
        name = self.get_name_from_person_id(person_id)
        response = self.add_user_message_and_get_response(f"<MEMBER JOINED> {name} has joined the group.")
        if response != "NONE":
            print("Error! GPT not returning NONE for <MEMBER JOINED> command.")
            print(response)
        else:
            return response

    def member_left_group(self, person_id: int):
        """
        Inform the GPT that someone has left the group.
        
        :param person_id: The ID for the person who has left the group.
        :returns response: Should be NONE
        """
        name = self.get_name_from_person_id(person_id)
        response = self.add_user_message_and_get_response(f"<MEMBER LEFT> {name} has left the group.")
        if response != "NONE":
            print("Error! GPT not returning NONE for <MEMBER LEFT> command.")
            print(response)
        else:
            return response
        
    def new_group(self, group_id: int, other_members: list):
        """
        Recall interactions that involved other members in the group.
        Requires looking through all of the interactions in the dict, extracting those
        where we have interacted with other group members, and then using a GPT to create
        a summary of these interactions with each person.  
        ... But for now we will just do <NEW GROUP> and give the GPT the names of the
        other members of the group.

        :param group_id: The ID number of the new group
        :param other_members: The ID numbers of tther members of the new group;
        this EXCLUDES this person.
        returns response: Should be NONE
        """
        other_members_names = [self.get_name_from_person_id(person) for person in other_members]
        response = self.add_user_message_and_get_response(f"<NEW GROUP> You have been moved to group {group_id}, \
        the other members of the group are: {other_members_names}")
        if response != "NONE":
            print("Error! GPT not returning NONE for <NEW GROUP> command.")
            print(response)
        else:
            return response

        # relevant_interactions = []
        # for interaction in self.interactions:
        #     if self.name in interaction: # TODO
        #         relevant_interactions.append(interaction)

    def other_member_text(
        self, 
        person_id: int, 
        group_id: int, 
        people_in_group: list, # Others in the group, EXCLUDES the speaker
        text: str
    ):
        """
        Update the interaction GPT with what another person 
        in the group has said.
        """
        # TODO JUST send these to the GPT rather than saving them?
        # But the saved list might be useful for a compressed memory version...
        # Every x minutes, send each person a compressed memory update summarising conversations had thus far ?
        # Maybe not needed as teh story-writing will be directing things enough that 
        # a poor memory won't be noticable ?
        name = self.get_name_from_person_id(person_id)
        response = self.add_user_message_and_get_response(f"<TEXT> {name} says: {text}")
        if response != "NONE":
            print("Error! GPT not returning NONE for <TEXT> command.")
            print(response)
        # Save the text to the interactions memory dict
        self.update_interactions_dict(person_id, group_id, people_in_group, text)

    def update_interactions_dict(
        self, 
        person_id: int, 
        group_id: int, 
        people_in_group: list, # Others in the group; EXCLUDES the speaker; id numbers not names
        text: str
    ):
        """
        Update a dict that keeps track of all things said in groups where
        this person is present.
        """
        formatted_now = datetime.now().strftime(self.datetime_format)
        self.interactions.append({
            'datetime': formatted_now, 
            'group_id': group_id,
            'speaker_id': person_id,
            'listener_ids': people_in_group,
            'text': text
        }) 

    def get_name_from_person_id(self, person_id: int):
        name = config.PERSON_INFO_DICT.get(person_id, {}).get('name', None), 
        if name == None:
            print("Error! Name not found for this person_id")
        return name

    ########## GPT METHODS ##########

    def create_assistant(self, intro_instructions):
        """
        Create the OpenAI ChatGPT assistant.
        """
        self.assistant = self.client.beta.assistants.create(
            name="person_{self.person_id}",
            instructions=intro_instructions,
            model="gpt-4o-mini",
            tools=[
                #{"type": "function", "function": self.image_analysis_json}
            ]
        )
        #self.show_json(self.assistant)

    def create_thread(self): #TODO I think as the thread gets longer, more tokens used, move money
        """
        Create a thread for the conversation with the GPT.
        """
        # The thread contains the whole conversation
        self.thread = self.client.beta.threads.create()
        #self.show_json(self.thread)

    def add_user_message_and_get_response(self, message: str):
        """
        Submit a message to the GPT and get a response back.
        """
        user_message = self.add_user_message(self.thread, message)
        completed = self.run(self.thread, self.assistant)
        print(f"completed {completed}")
        if completed == True:
            response = self.get_response(self.thread, user_message)
            # Convert response to a string
            response_str = str(response)
            # Define a regular expression pattern to find the value
            pattern = r'value=(?:"([^"]*)"|\'([^\']*)\')'
            # Search for the pattern in the response string
            matches = re.findall(pattern, response_str)
            # Extract the values from the matches
            extracted_values = [match[0] if match[0] else match[1] for match in matches]
            # If you want to get the first match only
            if extracted_values:
                first_value = extracted_values[0]
                print("First Extracted Value:", first_value)
            else:
                print("No value found in the response.")
            return first_value
        else:
            print("GPT run error!")

    def show_json(self, obj):
        print(json.loads(obj.model_dump_json()))

    def pretty_print(self, messages: list):
        print("# Messages")
        for m in messages:
            print(m.content)
            print(f"{m.role}: {m.content[0].text.value}")
        print()

    def add_user_message(self, thread, message: str):
        message = self.client.beta.threads.messages.create(
            thread_id=thread.id,
            role="user",
            content=message,
        )
        return message

    def run(self, thread, assistant):
        run = self.client.beta.threads.runs.create(
            thread_id=thread.id,
            assistant_id=assistant.id,
        )
        complete = self.wait_on_run(run, thread)
        return complete

    def wait_on_run(self, run, thread):
        while run.status == "queued" or run.status == "in_progress":
            run = self.client.beta.threads.runs.retrieve(
                thread_id=thread.id,
                run_id=run.id,
            )
            time.sleep(0.3)
        return True

    def get_response(self, thread, user_message):
        # Retrieve all the messages added after our last user message
        return self.client.beta.threads.messages.list(
            thread_id=thread.id, order="asc", after=user_message.id
        )

    
    
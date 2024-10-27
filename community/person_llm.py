
from openai import OpenAI
import json
import os
import re
import time
import requests
from datetime import datetime
import community.config_files.configuration as config

# interactions as a dict... a list of all the text they have exchanged with other people in order, with time stamps?
# or a list of dicts perhaps.  [{date_time: ''}, {group: ''}, {name: ''}, {text: ''}]
# - group 3 - 
# 2024-05-10-16-44-45 : group 3 : Jessica Rust : Hey how are you?
# 2024-05-10-16-45-17 : group 3 : Self : Fine thanks, how are you?


class PersonLLM():
    """
    The Person's GPT functonality - 
    Specific methods to request text from the LLM.
    The fine details of what prompt to give the LLM will be worked out by a PromptManager class in person_node
    """

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
        # Relationship object for each relationship!
        # Will need relationship objects to exist seperately somewhere... one object shared between two other objects and both
        # should be able to access and update it!
        # maybe these should be nodes?  relationship nodes?
        self.interactions = [] # create interactions list
        self.initialise_gpt()
        self.datetime_format = "%Y-%m-%d_%H-%M-%S"

    def initialise_gpt(self):
        """
        Create the GPT instance and feed it the intro_instructions and
        initial data and history for the person.
        """
        intro_instructions = f"You embody a human, \
            with a full life and backstory.  You will assigned a group, in which you will \
            be placed with 1 to 5 other people.  You will all have a conversation.\
            When a user message command is given, you should \
            only response with NONE, unless the command contains 'RESPOND', i.e. only give a real response \
            that is not NONE if the command is <RESPOND-JOINING>, <RESPOND-LEAVING>, or <RESPOND-NORMAL>; \
            this is very important.\n\
            If your response is not NONE, it should be slightly different every time.\n\
            Your response should be quite dramatic and emotive.\n\
            Use no more than about 30 words in a response.\n\
            You will be given one of these commands:\n\
            <NEW GROUP> You have joined a new group and are told who else is in your current group.\n\
            <TEXT> Another member of your group is speaking. You will be given their name, and \
            the text for what they say.\n\
            <MEMBER LEFT> You will be given the name of the person who has left the group.\n\
            <MEMBER JOINED> You will be given the name of the person who has joined the group.\n\
            <RESPOND> It's your turn to speak.  You will be given specific instructions on \
            what sort of thing you should say. \n\
            Before we start, here is some information about yourself.  \
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
            with a person largely guide how you interact with them if you are both in the same group \
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
        prompt_details: str
    ):
        """
        Request text from THIS PERSON.

        :param person_id: The ID for this person
        :param group_id: The ID for the group this person is in
        :param people_in_group: The other people in the group, 
        EXCLUDING this person.
        :param message_type: The type of message you want; 0=joining, 1=leaving, 2=normal convo.
        returns text: The text response from the GPT.
        :param directed_id: Who this message should be directed at.

        :returns reponse: The text response
        :returns gpt_message_id: The ID number of the GPT message reponse
        """
        response, gpt_message_id = self.add_user_message_and_get_response(f"<RESPOND> {prompt_details}")
        # Save the text to the interactions memory dict
        self.update_interactions_dict(person_id, group_id, people_in_group, response)
        return response, gpt_message_id

    def member_joined_group(self, person_id: int):
        """
        Inform the GPT that someone has joined the group.
        
        :param person_id: The ID for the person who has joined the group.
        :returns response: Should be NONE
        """
        name = self.get_name_from_person_id(person_id)
        response, message_id = self.add_user_message_and_get_response(f"<MEMBER JOINED> {name} has joined the group.")
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
        response, message_id = self.add_user_message_and_get_response(f"<MEMBER LEFT> {name} has left the group.")
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
        response, message_id = self.add_user_message_and_get_response(f"<NEW GROUP> You have been moved to group {group_id}, \
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
        response, message_id = self.add_user_message_and_get_response(f"<TEXT> {name} says: {text}")
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
            #print("response_str", response_str)
            # Define a regular expression pattern to find the value
            pattern = r'value=(?:"([^"]*)"|\'([^\']*)\')'
            # Search for the pattern in the response string
            matches = re.findall(pattern, response_str)
            # Extract the values from the matches
            extracted_values = [match[0] if match[0] else match[1] for match in matches]
            # If you want to get the first match only
            if extracted_values:
                first_value = extracted_values[0]
            #print("First Extracted Value:", first_value)
            else:
                print("No value found in the response.")
            # Define a regular expression pattern to find the id
            pattern = r'id=(?:"([^"]*)"|\'([^\']*)\')'
            # Search for the pattern in the response string
            matches = re.findall(pattern, response_str)
            # Extract the values from the matches
            extracted_values = [match[0] if match[0] else match[1] for match in matches]
            # If you want to get the first match only
            if extracted_values:
                first_id = extracted_values[0]
               # print("First Extracted ID:", first_id)
            else:
                print("No value found in the response.")
            return first_value, first_id
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

    def delete_messages_from_id(self, start_id: str): #TODO test
        """
        Delete all messages in the thread starting from the one with start_id.
        
        :param start_id: The message_id to start deleting from
        """
        # Step 1: Retrieve all messages in the thread
        messages = self.client.beta.threads.messages.list(
            thread_id=self.thread.id, 
            order="asc"  # Order the messages chronologically
        )
        
        # Step 2: Find the message with the given start_id
        delete_mode = False
        messages_to_delete = []

        for message in messages:
            if message.id == start_id:
                delete_mode = True  # Start deleting from this message onwards
            if delete_mode:
                messages_to_delete.append(message.id)
        
        # Step 3: Delete all the messages starting from start_id
        for message_id in messages_to_delete:
            self.delete_message(message_id)
            time.sleep(0.1)  # Adding a small delay to avoid rate limiting

    def delete_message(self, message_id: str):
        """ 
        Delete a specific message in a thread using HTTP DELETE request.
        
        :param message_id: The ID of the message to delete
        """
        url = f"https://api.openai.com/v1/threads/{self.thread.id}/messages/{message_id}"
        headers = {
            "Authorization": f"Bearer {self.api_key}",  # Use your OpenAI API key
            "Content-Type": "application/json",
            "OpenAI-Beta": "assistants=v2"  # Include the OpenAI Beta header
        }

        response = requests.delete(url, headers=headers)

        if response.status_code == 200:
            json_response = response.json()
            if json_response.get('deleted', False):
                print(f"Message {message_id} deleted successfully.")
            else:
                print(f"Message {message_id} not marked as deleted in the response.")
        else:
            print(f"Failed to delete message {message_id}. Status code: {response.status_code}, Response: {response.text}")


# Subscribe to faces_info topic
# Keep track of what system state we are in
# Publish state to system_state topic

import rclpy
from rclpy.node import Node

from community_interfaces.srv import (
    LlmTextRequest,
    LlmUpdateRequest,
    LlmRewindRequest
)
import community.configuration as config
from community.person_llm import PersonLLM
from community.prompt_manager import PromptManager
from community.helper_functions import HelperFunctions
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import yaml

from ament_index_python.packages import get_package_share_directory


class PersonNode(Node):

    def __init__(self):

        super().__init__('person_node')

        self.helper = HelperFunctions()

        # Get person_id from launch file -> long multi-digit number of RFID card
        self.declare_parameter('person_id', 0)
        self.person_id = self.get_parameter('person_id').get_parameter_value().integer_value

        # Now initialize the person object using person attributes from config yaml file
        person_data = self.helper.people_data.get(self.person_id, {})
        # Get voice id
        self.voice_id = str(person_data.get('voice_id', None))
        self.get_logger().info(f"Voice ID is: {self.voice_id}")
        if self.voice_id == None:
            self.get_logger().info("Error! Voice_id not found for this person_id")
        # Get color
        self.color = person_data.get('color', None)
        self.get_logger().info(f"Color is: {self.color}")
        if self.color == None:
            self.get_logger().info("Error! Color not found for this person_id")

        self.person = PersonLLM(
            person_id = self.person_id,
            name = person_data.get('name', "Person not found."),
            gender = person_data.get('gender', "Person not found."),
            age = person_data.get('age', "Person not found."),
            question = person_data.get('question', "Person not found."),
            history = person_data.get('history', "Person not found."),
            relationships = person_data.get('relationships', "Person not found."),
            personality = person_data.get('relationships', "Person not found.")
        )

        self.group_id = None # will change
        self.pi_id = None # will change
        self.group_members = [] # People in the group EXCLUDING this person

        # Initialise prompt manager
        self.prompt_manager = PromptManager(self.person_id)

        # Create callback group
        callback_group = MutuallyExclusiveCallbackGroup()

        # Initialise service servers
        self.srv = self.create_service(
            LlmTextRequest, 
            f'llm_text_request_{self.person_id}', 
            self.llm_text_request_callback,
            callback_group=callback_group
        )

    def llm_text_request_callback(self, request, response):
        """
        Callback function for requesting text from the GPT for this person.
        """
        self.get_logger().info('In llm_text_request_callback')
        prompt_details = self.prompt_manager.get_prompt_details(
            request.message_type, 
            request.directed_id, 
            request.event_id, 
            request.last_message,
            request.last_speaker_id,
            request.question_id,
            request.question_phase,
            request.mention_question
        )
        self.get_logger().info('Output prompt_details:')
        self.get_logger().info(str(prompt_details))
        text, gpt_message_id = self.person.person_speaks( # TODO get group_members from requst instead ?
            self.person_id,
            self.group_id,
            self.group_members, # Members of the group EXCLUDING the person who will talk.
            prompt_details
        )
        self.get_logger().info('GPT request complete')
        self.get_logger().info(f'Text from GPT: {text}')
        self.get_logger().info(f'Message ID from GPT: {gpt_message_id}')
        response.text = text
        response.gpt_message_id = gpt_message_id
        response.completed = True

        return response

    
def main(args=None):
    rclpy.init(args=args)

    person_node = PersonNode()
    # Use MultiThreadedExecutor to allow parallel execution
    executor = SingleThreadedExecutor()

    rclpy.spin(person_node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    person_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


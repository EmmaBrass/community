# Functions that were in the group node for processing relationship changes, that I have taken out for now.

def text_request_with_relationship(
        self, 
        seq,
        person_id, 
        directed_id, 
        event_id, 
        message_type, 
        question_id, 
        question_phase, 
        mention_question
    ):
    """
    Call the RelationshipManager service to tick on the relationship between two people,
    and return the current relationship state.
    
    :param person_a: The first person in the relationship.
    :param person_b: The second person in the relationship.
    :param group_id: The group that both people are in.
    :param group_members: All the people in the group (including person_a and person_b)
    
    :returns response.state_changed: bool, has the state changed?
    :returns from_state: what state we have changed from, if any
    :returns to_state: what state we have changed to, if any
    :returns action: What action to speak about, if any
    """
    self.get_logger().info("In text_request_with_relationship")

    # Create a request message
    request = RelationshipAction.Request()
    request.person_a = person_id
    request.person_b = directed_id
    request.group_id = self.group_id
    request.group_members = self.group_members

    # Send the request to the service and wait for the response
    future = self.tick_get_relationship_client.call_async(request)
    # rclpy.spin_until_future_complete(self, future)
    future.add_done_callback(lambda future: self.handle_tick_get_relationship_response(future, 
                                                                                        seq,
                                                                                        event_id, 
                                                                                        person_id, 
                                                                                        message_type, 
                                                                                        directed_id, 
                                                                                        question_id, 
                                                                                        question_phase,
                                                                                        mention_question
                                                                                        ))

# Define the callback function to handle the response:
def handle_tick_get_relationship_response(
        self, 
        future, 
        seq,
        event_id, 
        person_id, 
        message_type, 
        directed_id, 
        question_id, 
        question_phase,
        mention_question
    ):
    try:
        response = future.result()
        if response:
            self.get_logger().info(f"Received response: state changed: {response.state_changed}, from: {response.from_state}, to: {response.to_state}, action: {response.action}, tick_id: {response.tick_id}")
            # Check if the request was successful
            if future.result() is not None:
                response = future.result()
                msg = PersonTextRequest()
                msg.seq = seq
                msg.group_id = self.group_id
                msg.relationship_ticked = True
                msg.relationship_tick = response.tick_id
                msg.state_changed = response.state_changed
                msg.from_state = response.from_state
                msg.to_state = response.to_state
                msg.action = response.action
                msg.transition_description = response.transition_description
                msg.event_id = event_id
                msg.person_id = person_id
                msg.message_type = message_type
                msg.directed_id = directed_id 
                msg.question_id = question_id
                msg.question_phase = question_phase
                msg.mention_question = mention_question
                for i in range(5):
                    self.person_text_request_publisher.publish(msg)
                self.last_text_recieved = False
                self.creating_text_request = False
            else:
                self.get_logger().error("Failed to call tick_get_relationship service")
                
        else:
            self.get_logger().error("Received an empty response from tick_get_relationship service.")
    except Exception as e:
        self.get_logger().error(f"Service call failed with exception: {e}")

def call_rewind_relationship(self, group_id, tick_id, group_members):
    """
    Call the RelationshipManager service to rewind the relationships for this group to a given tick.
    
    :param group_id: The id of this group.
    :param tick_id: The tick to rewind to.
    :param group_members: The people in this group.
    
    :returns: True or False, whether the rewind request succeeded or not.
    """
    self.get_logger().info(f"Calling the RelationshipManager service to rewind the relationships for this group to a given tick.")
    # Create a request message
    request = RelationshipAction.Request()
    request.group_id = group_id
    request.tick_id = tick_id
    request.group_members = group_members

    # Send the request to the service and wait for the response
    future = self.rewind_relationship_client.call_async(request)
    future.add_done_callback(lambda future: self.call_rewind_relationship_callback(future))

def call_rewind_relationship_callback(self, future):
    # Check if the request was successful
    if future.result() is not None:
        response = future.result()
        self.get_logger().info(f"Rewind successful: {response.success}")
        success = True
    else:
        self.get_logger().error("Failed to call rewind_relationship service")
        success = False
    
    if success != True:
        self.get_logger().error("call_rewind_relationship failed!")
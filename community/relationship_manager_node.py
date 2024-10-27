import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from community_interfaces.srv import RelationshipAction  # Assuming you have a custom service defined
from community.relationship_manager import RelationshipManager

class RelationshipManagerService(Node):
    def __init__(self):
        super().__init__('relationship_manager_service')

        # Initialize the relationship manager
        self.relationship_manager = RelationshipManager()

        # Create services to manage relationships
        self.tick_get_relationship_service = self.create_service(RelationshipAction, 'tick_get_relationship', self.tick_get_relationship_callback)
        self.rewind_relationship_service = self.create_service(RelationshipAction, 'rewind_relationship', self.rewind_relationship_callback)

    def tick_get_relationship_callback(self, request, response):
        """
        Tick the relationship between two people,
        and return the state of that relationship.
        """
        person_a = request.person_a
        person_b = request.person_b
        group_id = request.group_id
        group_members = request.group_members
        result, tick_id = self.relationship_manager.tick_get_relationship(person_a, person_b, group_id, group_members) # state_changed, from, to, action
        response.state_changed = bool(result['state_changed'])
        response.from_state = str(result['from_state'])
        response.to_state = str(result['to_state'])
        response.action = str(result['action'])
        response.tick_id = tick_id
        return response

    def rewind_relationship_callback(self, request, response):
        """
        Rewind all relationships for a group back to a tick_id.
        """
        group_id = request.group_id
        tick_id = request.tick_id
        group_members = request.group_members
        response.success = self.relationship_manager.rewind_to_tick(group_id, group_members, tick_id)
        return response


def main(args=None):
    rclpy.init(args=args)

    relationship_manager_service = RelationshipManagerService()

    rclpy.spin(relationship_manager_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

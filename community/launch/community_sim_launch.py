from launch import LaunchDescription
from launch_ros.actions import Node
from community.configuration import GROUP_PI_ASSIGNMENTS
import os, yaml

# Define the path to the YAML file
print(os.path.dirname(__file__))
CONFIG_PATH = "/home/emma/community_ws/src/community/config_files/people.yaml" # TODO make this dynamic

def load_people():
    """Loads people data from the YAML file."""
    with open(CONFIG_PATH, 'r') as file:
        data = yaml.safe_load(file)
    return data.get('people', {})  # Returns dictionary of people

def generate_launch_description():
    nodes = [
        Node(
            package='community',
            executable='group_assignment_node',
            name='group_assignment_node',
            parameters=[
                {'log_level': 'INFO'}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        Node( # TODO dynamically create group nodes from the configuration.py file settings
            package='community',
            executable='group_node',
            name='group_node_1',
            parameters=[
                {'group_id': 1},
                {'log_level': 'INFO'}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        )
    ]

    # Load people from YAML
    people = load_people()

    # Dynamically create Person Nodes
    for person_id, person_info in people.items():
        nodes.append(
            Node(
                package='community',
                executable='person_node',
                name=f'person_node_{person_id}',
                output='screen',
                parameters=[
                    {'log_level': 'INFO'},
                    {'person_id': int(person_id)}
                ]
            )
        )
    
    # Dynamically create sim_pi_nodes based on GROUP_PI_ASSIGNMENTS
    for group_id, group_info in GROUP_PI_ASSIGNMENTS.items():
        for pi_id in group_info['pi_ids']:
            nodes.append(
                Node(
                    package='community',
                    executable='sim_pi_node',
                    name=f'sim_pi_node_{pi_id}',
                    output='screen',
                    parameters=[
                        {'log_level': 'INFO'},
                        {'pi_id': pi_id}
                    ]
                )
            )

    return LaunchDescription(nodes)

from launch import LaunchDescription
from launch_ros.actions import Node
from community.configuration import GROUP_PI_ASSIGNMENTS

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
        Node(
            package='community',
            executable='group_node',
            name='group_node_1',
            parameters=[
                {'group_id': 1},
                {'log_level': 'INFO'}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        Node(
            package='community',
            executable='relationship_manager_node',
            name='relationship_manager_node',
            output='screen',
            parameters=[
                {'log_level': 'INFO'}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        # Person Nodes
        Node(
            package='community',
            executable='person_node',
            name='person_node_1',
            output='screen',
            parameters=[
                {'log_level': 'INFO'},
                {'person_id': 17223924946}
            ]
        ),
        Node(
            package='community',
            executable='person_node',
            name='person_node_2',
            output='screen',
            parameters=[
                {'log_level': 'INFO'},
                {'person_id': 117227880}
            ]
        ),
        Node(
            package='community',
            executable='person_node',
            name='person_node_3',
            output='screen',
            parameters=[
                {'log_level': 'INFO'},
                {'person_id': 5299113}
            ]
        ),
        Node(
            package='community',
            executable='person_node',
            name='person_node_4',
            output='screen',
            parameters=[
                {'log_level': 'INFO'},
                {'person_id': 6024224946}
            ]
        )
    ]

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

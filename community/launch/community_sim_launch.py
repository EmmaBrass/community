from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# TODO automate creation of this launch file from configuration.py and people.yaml

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='community',
            executable='group_assignment_node',
            name='group_assignment_node',
            parameters=[
                {'log_level': 'INFO'}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),

        # Group Node 1
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

        # Person Node 1
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
        # Person Node 2
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
        # Person Node 3
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
        # Person Node 4
        Node(
            package='community',
            executable='person_node',
            name='person_node_4',
            output='screen',
            parameters=[
                {'log_level': 'INFO'},
                {'person_id': 6024224946}
            ]
        ),

         # Sim Pi Node 1
        Node(
            package='community', 
            executable='sim_pi_node',
            name='pi_node_1',
            output='screen',
            parameters=[
                {'log_level': 'INFO'},
                {'pi_id': 1}
            ]
        ),
        # Sim Pi Node 2
        Node(
            package='community', 
            executable='sim_pi_node',
            name='pi_node_2',
            output='screen',
            parameters=[
                {'log_level': 'INFO'},
                {'pi_id': 2}
            ]
        ),
        # Sim Pi Node 3
        Node(
            package='community',
            executable='sim_pi_node',
            name='pi_node_3',
            output='screen',
            parameters=[
                {'log_level': 'INFO'},
                {'pi_id': 3}
            ]
        ),
        # Sim Pi Node 4
        Node(
            package='community',
            executable='sim_pi_node',
            name='pi_node_4',
            output='screen',
            parameters=[
                {'log_level': 'INFO'},
                {'pi_id': 4}
            ]
        ),


    ])
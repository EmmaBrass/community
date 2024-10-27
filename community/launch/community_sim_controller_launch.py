from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# TODO automate creation of this launch file from configuration.py and people.yaml

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='community',
            executable='sim_pi_controller_node',
            name='sim_pi_controller',
            output='screen',
            parameters=[
                {'log_level': 'INFO'}
            ]
        ),

    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='community',
            executable='pi_node',
            name='pi_node',
            parameters=[
                {'pi_id': '1'}, # customise for each pi (should match comp name, e.g. head1, head2)
                {'log_level': 'INFO'}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        )
    ])
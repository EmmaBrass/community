from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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
                {'group_id': '1'},
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
                {'person_id': 1},
                {'name': 'Jessica Rust'},

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
                {'person_id': 2},
                {'name': 'Steve Carrlec'},
                {'age': 35},
                {'openness': 65},
                {'conscientiousness': 70},
                {'neuroticism': 90},
                {'agreeableness': 45},
                {'extraversion': 20},
                {'history': 'I am a physisict.'},
                {'relationships': {
                    'Jessica Rust' : 'My wife.',
                    'John Bravo' : 'Met him once.  Found him to be abrasive.',
                    'Margo Stewart' : 'Like her a lot; her sister\'s dementia makes me feel existential.'
                    }
                }
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
                {'person_id': 3},
                {'name': 'John Bravo'},
                {'age': 29},
                {'openness': 65},
                {'conscientiousness': 40},
                {'neuroticism': 30},
                {'agreeableness': 70},
                {'extraversion': 90},
                {'history': 'I recently moved to Australia and I love it here.'},
                {'relationships': {
                    'Jessica Rust' : 'My ex girlfriend.',
                    'Steve Carrlec' : 'Met him once.  He is a nice guy',
                    'Margo Stewart' : 'Never met.'
                    }
                }
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
                {'person_id': 4},
                {'name': 'Margo Stewart'},
                {'age': 53},
                {'openness': 90},
                {'conscientiousness': 65},
                {'neuroticism': 35},
                {'agreeableness': 80},
                {'extraversion': 55},
                {'history': 'My sister has dementia.'},
                {'relationships': {
                    'Jessica Rust' : 'My niece.',
                    'Steve Carrlec' : 'Like him a lot, he is friendly.',
                    'John Bravo' : 'Never met.'
                    }
                }
            ]
        )
    ])
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='leap_node',
            executable='hands_publisher',
            output='screen'
        ),
        Node(
            package='ohrc_leap',
            executable='state_topic_publisher',
            output='screen',
        ),
    ])
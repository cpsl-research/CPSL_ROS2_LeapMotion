from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare(
                'ohrc_teleoperation'), '/launch/state_topic_teleoperation.launch.py']),
        ),

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
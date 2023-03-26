from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():

    whisper_node = Node(
        package="ros2_whisper",
        executable="whisper_node",
        output="screen",
    )

    return LaunchDescription([whisper_node])


from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    initial_calibration = LaunchConfiguration("initial_calibration")
    initial_calibration_cmd = DeclareLaunchArgument(
        "initial_calibration",
        default_value="true",
        description="Wheter to perform an initial calibration")

    dynamic_energy = LaunchConfiguration("dynamic_energy")
    dynamic_energy_cmd = DeclareLaunchArgument(
        "dynamic_energy",
        default_value="true",
        description="Wheter to recalibrate dynamically")

    whisper_model = LaunchConfiguration("whisper_model")
    whisper_model_cmd = DeclareLaunchArgument(
        "whisper_model",
        default_value="small",
        description="Whisper model",
        choices=["tiny", "base", "small", "medium", "large"])

    whisper_node = Node(
        package="whisper_ros",
        executable="whisper_node",
        output="screen",
        parameters=[{"initial_calibration": initial_calibration},
                    {"dynamic_energy": dynamic_energy},
                    {"whisper_model": whisper_model}]
    )

    return LaunchDescription([
        initial_calibration_cmd,
        dynamic_energy_cmd,
        whisper_model_cmd,
        whisper_node
    ])

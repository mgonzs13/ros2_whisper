# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


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

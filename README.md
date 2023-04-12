# whisper_ros

Repository showcases inference of the [OpenAI Whisper](https://github.com/openai/whisper) within a ROS 2 node.

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/mgonzs13/whisper_ros.git
$ pip3 install -r requirements.txt
$ cd ~/ros2_ws
$ colcon build
```

## Usage

```shell
$ ros2 launch whisper_bringup whisper.launch.py
```

To print the inferenced text, do

```shell
$ ros2 topic echo /whisper_text
```

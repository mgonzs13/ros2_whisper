# ROS2 Whisper

Repository showcases inference of the [OpenAI Whisper](https://github.com/openai/whisper) within a ROS 2 node.

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/mgonzs13/ros2_whisper.git
$ pip3 install -r requirements.txt
$ cd ~/ros2_ws
$ colcon build
```

## Usage

```shell
$ ros2 launch ros2_whisper ros2_whisper.launch.py
```

To print the inferenced text, do

```shell
$ ros2 topic echo /whisper/text
```

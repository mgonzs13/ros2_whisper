
import torch
import whisper
import numpy as np
import speech_recognition as sr

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String


class WhisperNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        # params
        self.declare_parameters(
            namespace="",
            parameters=[
                ("initial_calibration", True),
                ("dynamic_energy", True),
                ("whisper_model", "small")
            ]
        )

        if self.get_parameter("initial_calibration").get_parameter_value().bool_value:
            energy = self.calibrate_stt(2)
        else:
            energy = 300

        # speech_recognition
        self.r = sr.Recognizer()
        self.r.energy_threshold = energy
        self.r.dynamic_energy_threshold = self.get_parameter(
            "dynamic_energy").get_parameter_value().bool_value

        # whisper model
        self.whisper_model = whisper.load_model(self.get_parameter(
            "whisper_model").get_parameter_value().string_value)
        device = "cpu"
        if torch.cuda.is_available():
            self.get_logger().info("CUDA is available. Using GPU.")
            device = "cuda"
        self.whisper_model = self.whisper_model.to(device)

        # pub
        self.text_pub = self.create_publisher(
            String, "whisper/text", qos_profile_system_default)

        # threads
        self.create_timer(1, self.work)
        self.get_logger().info("Whisper Started")

    def calibrate_stt(self, seconds: int):

        rec = sr.Recognizer()
        mic = sr.Microphone()
        self.get_logger().info("A moment of silence, please...")

        with mic as source:
            rec.adjust_for_ambient_noise(source, duration=seconds)

        self.get_logger().info(
            f"Set minimum energy threshold to { rec.energy_threshold}")
        return rec.energy_threshold

    def work(self) -> None:

        with sr.Microphone(sample_rate=16000) as source:

            self.get_logger().info("Listening")
            audio = self.r.listen(source)

            audio_data = torch.from_numpy(np.frombuffer(
                audio.get_raw_data(), np.int16).flatten().astype(np.float32) / 32768.0)

            result = self.whisper_model.transcribe(audio_data)

            msg = String(data=result["text"])
            self.text_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperNode("whisper_node")
    rclpy.spin(whisper_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

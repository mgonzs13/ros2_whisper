
import queue
import torch
import whisper
import numpy as np
from threading import Thread
import speech_recognition as sr

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String


class WhisperNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        # params
        self.energy = self.calibrate_stt(2)
        self.pause = 0.8
        self.dynamic_energy = True

        # whisper model
        self.whisper_model = whisper.load_model("small")
        device = "cpu"
        if torch.cuda.is_available():
            self.get_logger().info("CUDA is available. Using GPU.")
            device = "cuda"
        self.whisper_model = self.whisper_model.to(device)

        # pub
        self.audio_queue = queue.Queue()
        self.text_pub = self.create_publisher(
            String, "whisper/text", qos_profile_system_default)

        # threads
        Thread(target=self.record_audio).start()
        Thread(target=self.transcribe_forever).start()

    def calibrate_stt(self, seconds: int):

        rec = sr.Recognizer()
        mic = sr.Microphone()
        self.get_logger().info("A moment of silence, please...")

        with mic as source:
            rec.adjust_for_ambient_noise(source, duration=seconds)

        self.get_logger().info(
            f"Set minimum energy threshold to { rec.energy_threshold}")
        return rec.energy_threshold

    def record_audio(self) -> None:
        r = sr.Recognizer()
        r.energy_threshold = self.energy
        r.pause_threshold = self.pause
        r.dynamic_energy_threshold = self.dynamic_energy

        with sr.Microphone(sample_rate=16000) as source:
            while True:
                audio = r.listen(source)
                audio_data = torch.from_numpy(np.frombuffer(
                    audio.get_raw_data(), np.int16).flatten().astype(np.float32) / 32768.0)

                self.audio_queue.put_nowait(audio_data)

    def transcribe_forever(self):
        while True:
            audio_data = self.audio_queue.get()
            result = self.whisper_model.transcribe(audio_data)

            msg = String(data=result["text"])
            self.text_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    whisper_inference_node = WhisperNode("whisper_inference_node")
    rclpy.spin(whisper_inference_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

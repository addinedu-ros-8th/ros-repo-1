import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String

import sounddevice as sd
import numpy as np
import queue
import os

from google.cloud import speech_v1 as speech
from google.cloud.speech_v1 import types

class MicrophoneStream:
    def __init__(self, rate, chunk_size):
        self.rate = rate
        self.chunk = chunk_size
        self.q = queue.Queue()
        self.stream = None

    def __enter__(self):
        self.stream = sd.InputStream(
            samplerate=self.rate,
            blocksize=self.chunk,
            dtype='int16',
            channels=1,
            callback=self.callback,
        )
        self.stream.start()
        return self

    def __exit__(self, type, value, traceback):
        if self.stream:
            self.stream.stop()
            self.stream.close()

    def callback(self, indata, frames, time, status):
        self.q.put(bytes(indata))

    def generator(self):
        while True:
            data = self.q.get()
            if data is None:
                return
            yield data


class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')

        self.key_file_path = "assistant.json"
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.key_file_path
        self.client = speech.SpeechClient()

        # Audio config
        self.sample_rate = 16000
        self.chunk_size = int(self.sample_rate / 10)  # 100ms

        # ROS pub/sub
        self.init_publisher = self.create_publisher(String, 'llm_init_state', 0)
        self.llm_state_publisher = self.create_publisher(String, 'llm_state', 0)
        self.llm_state_subscriber = self.create_subscription(String, 'llm_state', 
                                                             self.state_listener_callback, 0)
        self.audio_to_text_publisher = self.create_publisher(String, 'llm_input', 10)

        self.publish_string("llm_audio_input", self.init_publisher)

    def state_listener_callback(self, msg):
        if msg.data == "listening":
            self.get_logger().info(f"STATE: {msg.data}")
            self.action_function_listening()

    def action_function_listening(self):
        self.get_logger().info(" 음성 인식 시작...")
        self.publish_string("input_processing", self.llm_state_publisher)

        config = types.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=self.sample_rate,
            language_code="ko-KR",
            enable_automatic_punctuation=True,
        )

        streaming_config = types.StreamingRecognitionConfig(
            config=config,
            interim_results=False
        )

        with MicrophoneStream(self.sample_rate, self.chunk_size) as stream:
            audio_generator = stream.generator()
            requests = (
                types.StreamingRecognizeRequest(audio_content=chunk)
                for chunk in audio_generator
            )

            try:
                responses = self.client.streaming_recognize(streaming_config, requests)
                for response in responses:
                    if not response.results:continue
                    result = response.results[0]
                    
                    if not result.alternatives:continue
                    transcript = result.alternatives[0].transcript.strip()

                    self.get_logger().info(f" 인식 결과: {transcript}")
                    self.publish_string(transcript, self.audio_to_text_publisher)
                    break  # 한 문장만 처리
            except Exception as e:
                self.get_logger().error(f"STT Error: {e}")
                self.publish_string("listening", self.llm_state_publisher)

    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send
        publisher_to_use.publish(msg)
        self.get_logger().info(
            f" Topic: {publisher_to_use.topic_name}\n Message: {msg.data}"
        )


def main(args=None):
    rclpy.init(args=args)
    stt = STTNode()
    rclpy.spin(stt)
    stt.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

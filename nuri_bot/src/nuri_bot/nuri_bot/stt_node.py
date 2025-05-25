# ros libs
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# google stt api
from google.cloud import speech_v1 as speech
from google.cloud.speech_v1 import types
# extra
import sounddevice as sd
import numpy as np
import queue
import os
import time
import threading
# Global Initialization
from services.srv import Trigger
from pinky_interfaces.srv import Emotion


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
    def __init__(self, config):
        super().__init__('stt_node')
        self.key_file_path = "assistant.json"
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.key_file_path
        self.client = speech.SpeechClient()
        self.config = config
        # Audio config
        self.sample_rate = config.sample_rate
        self.chunk_size = int(config.sample_rate / 10)  # 100ms

        # ROS pub/sub
        # self.init_publisher = self.create_publisher(String, 'nuri_init_state', 0)
        self.llm_state_publisher = self.create_publisher(String, 'llm_state', 0)
        # self.llm_state_subscriber = self.create_subscription(String, 'nuri_state',
        #                                                      self.state_listener_callback, 0)

        self.__pub_to_llm = self.create_publisher(String, '/llm_input', 10)
        self.__pub_to_handler= self.create_publisher(String, '/response_walk', 10)
        self.__pub_bye= self.create_publisher(String, '/bye', 10)

        # self.publish_string("llm_audio_input", self.init_publisher)

        self.trigger_client = self.create_client(Trigger, '/trigger')
        while not self.trigger_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.__srv_emotion = self.create_client(Emotion, "set_emotion")


        self.response_timer = None
        self.is_listening = False
        self.trigger_word = config.trigger_word

        self.exit_keywords = ["끝", "그만", "잘가", "바이바이", "종료", "안녕히", "이제 그만", "너 일 봐라"]
    def run(self):
        self.is_trigger_active = True
        self.trigger_thread = threading.Thread(target=self.trigger_detector)
        self.trigger_thread.daemon = True
        self.trigger_thread.start()
        self.get_logger().info("Trigger word active")

        # face expression
        req = Emotion.Request()
        req.emotion = "basic"
        self.__srv_emotion.call_async(req)

    def trigger_detector(self):
        self.get_logger().info("트리거 단어 감지 시작 (Google STT)...")
        while rclpy.ok():
            if not self.is_listening:
                transcript_generator = self.process_audio_stream(interim_results=True)
                try:
                    for transcript in transcript_generator:
                        if transcript and self.trigger_word.lower() in transcript:
                            self.get_logger().info(f"트리거 단어 '{self.trigger_word}' 감지! (Google STT)")
                            # request service
                            req = Trigger.Request()
                            req.text_to_speak = self.config.trigger_response
                            future = self.trigger_client.call_async(req)

                            # wait for service response
                            rclpy.spin_until_future_complete(self, future)
                            if future.result() is not None:
                                response = future.result()
                                if response.success:
                                    self.get_logger().info("Trigger 응답 끝남을 수신")
                                    # For now, listen
                                    self.action_function_listening()
                                else:
                                    self.get_logger().error("TTS 서비스 요청 실패.")
                            else:
                                self.get_logger().error('TTS 서비스 요청 응답 없음.')
                            break  # 트리거 단어 감지 후 인식 종료
                        time.sleep(0.1) # CPU 사용률 감소
                except Exception as e:
                    self.get_logger().error(f"트리거 감지 중 오류: {e}")
            else:
                time.sleep(0.5) # 이미 리스닝 중이면 잠시 대기

    def action_function_listening(self):
        self.is_listening = True
        self.publish_string("input_processing", self.llm_state_publisher)
        transcript_generator = self.process_audio_stream(interim_results=False)

        try:
            for transcript in transcript_generator:
                if transcript:
                    self.get_logger().info(f" 인식 결과: {transcript}")

                    # 대화 종료 키워드 체크
                    if any(keyword in transcript for keyword in self.exit_keywords):
                        self.get_logger().info("대화 종료 키워드 감지")
                        self.end_conversation()
                        return
                    
                    self.publish_string(transcript, self.__pub_to_llm)
                    break

        except Exception as e:
            self.get_logger().error(f"STT Error in action_function_listening: {e}")
            self.is_listening = False
            self.publish_string("listening", self.llm_state_publisher)

    def end_conversation(self):
        """대화 종료"""
        self.get_logger().info("대화를 종료합니다")
        self.is_listening = False
        
        # 기본 표정으로 복귀
        req = Emotion.Request()
        req.emotion = "basic"
        self.__srv_emotion.call_async(req)

        self.publish_string("bye", self.__pub_bye)

                    


        
    def process_audio_stream(self, interim_results=False):
        config = types.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=self.sample_rate,
            language_code="ko-KR",
            enable_automatic_punctuation=True,
        )
        streaming_config = types.StreamingRecognitionConfig(
            config=config,
            interim_results=interim_results
        )

        with MicrophoneStream(self.sample_rate, self.chunk_size) as stream:
            audio_generator = stream.generator()
            requests = (types.StreamingRecognizeRequest(audio_content=chunk) for chunk in audio_generator)

            try:
                responses = self.client.streaming_recognize(streaming_config, requests)
                for response in responses:
                    if not response.results:
                        continue
                    result = response.results[0]
                    if not result.alternatives:
                        continue
                    transcript = result.alternatives[0].transcript.strip().lower()
                    yield transcript
            except Exception as e:
                self.get_logger().error(f"Google STT Error: {e}")
                yield None



    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send
        publisher_to_use.publish(msg)

        self.get_logger().info(f" Topic: {publisher_to_use.topic_name}, Message: {msg.data}")



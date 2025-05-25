import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from gtts import gTTS
import os
import tempfile
import playsound 
import time
import threading 

from services.srv import Trigger 
from pinky_interfaces.srv import Emotion

class TTSNode(Node):
    def __init__(self, config):
        super().__init__('tts_node')
        self.config = config 
        
        # Initialization publisher
        self.nuri_init_publisher = self.create_publisher(String, "/nuri_init_state", 0)
        # Nuri state publisher
        self.llm_state_publisher = self.create_publisher(String, "/llm_state", 0)
        
        # Feedback from LLM 
        self.feedback_for_user_subscriber = self.create_subscription(           
            String, "/llm_feedback_to_user", self.feedback_for_user_callback, 10)


        self.trigger_service = self.create_service(
            Trigger,'/trigger',self.trigger_callback)
        
        # tts 완료 시그널 
        self.__pub_speech_complete = self.create_publisher(
            String,'/speech_complete',10)

        
        self.__srv_emotion = self.create_client(Emotion, "set_emotion")
        
        self.current_context = "conversation"  # 현재 TTS 컨텍스트
        # 초기화 시 워밍업 사운드 재생
        self.warm_up_audio_system()
        
       
    def warm_up_audio_system(self):
        self.get_logger().info("오디오 시스템 워밍업 시작...")
        # 짧은 무음 또는 아주 작은 소리를 재생하여 시스템 초기화
        try: 
            # gTTS로 짧은 공백 텍스트를 음성으로 변환하여 재생 (약 0.1초)
            tts = gTTS(text="네", lang='ko')
            warmup_file_path = "/tmp/tts_warmup.mp3"
            tts.save(warmup_file_path)
            playsound.playsound(warmup_file_path, block=False) # 비동기로 재생하여 초기화만
            time.sleep(0.1) # 재생 시작을 위한 짧은 대기
            os.remove(warmup_file_path)
            self.get_logger().info("오디오 시스템 워밍업 완료.")
            self.get_logger().info("TTSNode 초기화 완료.") # 추가

        except Exception as e:
            self.get_logger().error(f"오디오 워밍업 중 오류 발생: {e}")

    def tts_speak(self, text, speak_type):
        if not text: 
            return
        try:
            self.get_logger().info(f"🔊 TTS 시작: {text}")
            tts = gTTS(text=text, lang='ko')
            with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
                tts.save(fp.name)
                playsound.playsound(fp.name)
                os.unlink(fp.name)
            if speak_type == "trigger_talk":
                pass
            elif speak_type == "bye":
                pass     
            else:
                msg = String()
                msg.data = "spoken"
                self.__pub_speech_complete.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"gTTS 오류: {e}")


    def trigger_callback(self, request, response):
        """
            stt로 부터 받은 trigger 응답 요청을 처리하는 service callback 함수 
        """
        self.current_context = "trigger"

        # face expression 
        req = Emotion.Request()
        req.emotion = "greeting"
        self.__srv_emotion.call_async(req)

        # 2. audio play 
        self.tts_speak(request.text_to_speak, "trigger_talk")

        # 4. service response 
        response.success = True
        return response
    
    def feedback_for_user_callback(self, msg):
        self.get_logger().info("Received text: '%s'" % msg.data)

        # 대화 컨텍스트 설정
        self.current_context = "conversation"
        
        raw = msg.data 
        emotion, text = self.parse_llm_emotion_response(raw)

        # adjust LCD emotion 
        req = Emotion.Request()
        req.emotion = emotion  # "sad"
        self.__srv_emotion.call_async(req)  # sad.gif 표시
        
        # output answer 
        self.tts_speak(text, "llm")



    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send

        publisher_to_use.publish(msg)
        self.get_logger().info(f"Topic: {publisher_to_use.topic_name}\nMessage published: {msg.data}")


    def parse_llm_emotion_response(self, response_text):
        """
        LLM 응답에서 감정과 내용 분리 (대괄호 제거 포함)
        """
        try:
            if '|' in response_text:
                parts = response_text.split('|', 1)
                if len(parts) == 2:
                    raw_emotion = parts[0].strip()  # "[sad]" 또는 "sad"
                    text = parts[1].strip()         # "많이 아프시겠어요..."
                    
                    # 대괄호 제거 처리
                    if raw_emotion.startswith('[') and raw_emotion.endswith(']'):
                        emotion = raw_emotion[1:-1].strip()  # "[sad]" → "sad"
                    else:
                        emotion = raw_emotion  # 이미 "sad" 형태
                    
                    # 유효한 감정인지 확인
                    valid_emotions = ['angry', 'happy', 'sad', 'fun', 'interest', 'greeting', 'board', 'basic']
                    if emotion in valid_emotions:
                        return emotion, text
                    else:
                        self.get_logger().warning(f"유효하지 않은 감정: {emotion}")
                        
            # 파싱 실패 시 기본값
            self.get_logger().warning("감정 파싱 실패, 기본 감정 사용")
            return 'basic', response_text
            
        except Exception as e:
            self.get_logger().error(f"감정 파싱 오류: {e}")
            return 'basic', response_text


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os
import tempfile
import playsound 
import time
import threading 
# LCD
from .pinky_lcd import LCD
from .lcd_controller import LCDController

# Global Initialization
from trigger.srv import TriggerResponse 
from config.config_node import UserConfig
config = UserConfig()

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        # Initialization publisher
        self.init_publisher = self.create_publisher(String, "/llm_init_state", 0)
        # LLM state publisher
        self.llm_state_publisher = self.create_publisher(String, "/llm_state", 0)
         # Feedback for user listener
        self.feedback_for_user_subscriber = self.create_subscription(
            String, "/llm_feedback_to_user", self.feedback_for_user_callback, 10)
        # Trigger detection sub         
        self.trigger_service = self.create_service(
            TriggerResponse,
            'trigger',
            self.trigger_callback
        )
        # LCD 
        self.lcd_controller = LCDController(node_logger=self.get_logger())
        # 초기화 시 워밍업 사운드 재생
        self.warm_up_audio_system()

    def warm_up_audio_system(self):
        self.get_logger().info("오디오 시스템 워밍업 시작...")
        # 짧은 무음 또는 아주 작은 소리를 재생하여 시스템 초기화
        try: 
            # gTTS로 짧은 공백 텍스트를 음성으로 변환하여 재생 (약 0.1초)
            tts = gTTS(text="음", lang='ko')
            warmup_file_path = "/tmp/tts_warmup.mp3"
            tts.save(warmup_file_path)
            playsound.playsound(warmup_file_path, block=False) # 비동기로 재생하여 초기화만
            time.sleep(0.1) # 재생 시작을 위한 짧은 대기
            os.remove(warmup_file_path)
            self.get_logger().info("오디오 시스템 워밍업 완료.")
        except Exception as e:
            self.get_logger().error(f"오디오 워밍업 중 오류 발생: {e}")

    def trigger_callback(self, request, response):
        """
            stt로 부터 받은 trigger 응답 요청을 처리하는 service callback 함수 
        """
        self.get_logger().info(f"서비스 요청 수신: '{request.text_to_speak}'")
        
        if request.text_to_speak == config.trigger_response:
            # 1. GIF play 
            gif_path = "/home/pinky/pinky_moon/src/example/example.gif"
            gif_thread = threading.Thread(target=self.lcd_controller.play_gif,
                                        args=(gif_path, 0.05))
            gif_thread.daemon = True # 메인 스레드 종료 시 서브 스레드도 종료
            gif_thread.start()
            
            # 2. audio play 
            self.tts_speak(request.text_to_speak)
        
        # 피드백 끝나고 LCD 화면 지우기 (선택 사항)
        self.lcd_controller.clear_lcd()
        # 4. service response 
        response.success = True
        return response
    def feedback_for_user_callback(self, msg):
        self.get_logger().info("Received text: '%s'" % msg.data)
        
        self.tts_speak(msg.data.strip())
        self.publish_string("feedback finished", self.llm_state_publisher)
        self.publish_string("listening", self.llm_state_publisher)
        # 피드백 끝나고 LCD 화면 지우기 (선택 사항)
        self.lcd_controller.clear_lcd()
    
    def tts_speak(self, text):
        if not text:
            return
        try:
            tts = gTTS(text=text, lang='ko')
            with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
                tts.save(fp.name)
                playsound.playsound(fp.name)
                os.unlink(fp.name)

        except Exception as e:
            self.get_logger().error(f"gTTS 오류: {e}")


    # def feedback_for_user_callback(self, msg):
    #     self.get_logger().info("Received text: '%s'" % msg.data)

    #     text = msg.data.strip()
    #     if not text: return 
    #     try:
    #         tts = gTTS(text=text, lang='ko')
    #         with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
    #             tts.save(fp.name)
    #             playsound.playsound(fp.name)
    #             os.unlink(fp.name) 
    #             self.publish_string("feedback finished", self.llm_state_publisher)
    #             self.publish_string("listening", self.llm_state_publisher)
    #     except Exception as e:
    #         self.get_logger().error(f"gTTS 오류: {e}")

    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send

        publisher_to_use.publish(msg)
        self.get_logger().info(
            f"Topic: {publisher_to_use.topic_name}\nMessage published: {msg.data}"
        )
    


def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()
    rclpy.spin(tts_node)
    tts_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
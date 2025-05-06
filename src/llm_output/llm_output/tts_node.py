import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os
import tempfile
import playsound 

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


    def feedback_for_user_callback(self, msg):
        self.get_logger().info("Received text: '%s'" % msg.data)

        text = msg.data.strip()
        if not text: return 
        try:
            tts = gTTS(text=text, lang='ko')
            with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
                tts.save(fp.name)
                playsound.playsound(fp.name)
                os.unlink(fp.name) 
                self.publish_string("feedback finished", self.llm_state_publisher)
                self.publish_string("listening", self.llm_state_publisher)
        except Exception as e:
            self.get_logger().error(f"gTTS 오류: {e}")
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



import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
# Nuri nodes 
from .handler import Handler 
from .stt_node import STTNode
from .tts_node import TTSNode
from .cam_node import CamNode


# Global Initialization
from config.config_node import UserConfig

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.get_logger().info("MainNode 초기화 시작.")
        self.config = UserConfig()

        self.cam = CamNode(self.config)
        self.stt = STTNode(self.config)
        self.tts = TTSNode(self.config)
        self.walk = WalkNode()
        
        self.handler = Handler(self, self.cam, self.stt, self.tts)

        
        
def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    main_node = MainNode()
    executor.add_node(main_node)
    executor.add_node(main_node.cam)
    executor.add_node(main_node.stt)
    executor.add_node(main_node.tts)
    executor.add_node(main_node.walk)
    try:
        executor.spin()
    finally:
        main_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

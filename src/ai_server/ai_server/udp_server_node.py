
# ROS reated libraries
import rclpy
from rclpy.node import Node

# vision related libraries
import socket
import numpy as np
import struct  # 추가: 데이터 크기 처리를 위해 
import cv2
# Global Initialization
from emergency.srv import EmergencyReport   # service 
from config.config_node import UserConfig
config = UserConfig()

class UdpImageReceiver(Node):
    def __init__(self):
        super().__init__('udp_image_receiver')
        self.udp_address = ('0.0.0.0', 5000)  # 수신할 주소 및 포트 (모든 인터페이스에서 수신)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.udp_address)
        # init service network
        self.emergency_client = self.create_client(EmergencyReport,
                                                    'report_emergency')
        while not self.emergency_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.logger = self.get_logger() 
        self.timer = self.create_timer(config.frame_rate, self.receive_frame)  # 30Hz로 수신 처리

    def receive_frame(self):
        try:
            # 먼저 이미지 크기를 수신 (4바이트, unsigned int)
            size_bytes, _ = self.sock.recvfrom(4)
            img_size = struct.unpack("<I", size_bytes)[0]  # little-endian unsigned int
            # robot_id 수신 (8바이트, signed int64)
            robot_id_bytes, _ = self.sock.recvfrom(8)
            robot_id = struct.unpack("<q", robot_id_bytes)[0]

            img_bytes, _ = self.sock.recvfrom(img_size) # 이미지 데이터 수신
            img_np = np.frombuffer(img_bytes, dtype=np.uint8)
            frame = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
            
            if frame is not None:
                cv2.imshow(f'Received Frame (UDP) - Robot: {robot_id}', frame)
                cv2.waitKey(1)
                self.detect_emergency(frame, robot_id)  # 긴급 상황 감지 함수 호출
            else:
                self.logger.warn('Failed to decode image')

        except socket.error as e:
            self.logger.error(f'Socket error: {e}')
        except struct.error as e:
            self.logger.error(f'Struct error: {e}')
        except Exception as e:
            self.logger.error(f'Error receiving frame: {e}')
    
    def detect_emergency(self, frame, robot_id):
        """
            긴급 상황 감지 로직을 구현합니다. 
        """
        self.logger.info(f"Emergency detected for robot {robot_id}!")
    
    def send_emergency (self, robot_id, emergency_type):
        """
            긴급 상황 발생 시, tcp_server_node에 알림을 전송합니다.    
        """
        req = EmergencyReport.Request()
        self.robot_id = robot_id
        req.emergency_type = emergency_type
        self.emergency_client.call_async(req)
        self.logger.info(f'Sent emergency to TCP server via service: Robot {robot_id} - {emergency_type}')

    def destroy_node(self):
        self.sock.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpImageReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
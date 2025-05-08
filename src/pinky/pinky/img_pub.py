"""
    pinky generate frames from camera and forward them to the server. 
"""

import cv2 
import socket 
# vision related libraries
from cv_bridge import CvBridge 
import struct
# ROS related libraries
import rclpy
from rclpy.node import Node
# from sensor_msgs.msg import Image
from config.config_node import UserConfig
# Global Initialization
config = UserConfig()

class CamUdpPub(Node): 
    def __init__(self):
        super().__init__('cam_udp_pub')
        self.bridge = CvBridge()     # convert between ROS and OpenCV images
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('camera not opened')
            self.destroy_node()
            return 
        
        self.udp_address = (config.server_ip, config.server_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.timer = self.create_timer(config.frame_rate, self.send_frame)  # 30 FPS
        self.robot_id = 1
    def send_frame(self):
        ret, frame = self.cap.read()
        if ret:  # Corrected condition
            try:
                _, img_encoded = cv2.imencode(
                    '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])  # Encoding parameters
                if _: # Check if encoding was successful
                    img_bytes = img_encoded.tobytes()
                    img_len = len(img_bytes)
                    img_len_bytes = struct.pack("<I", img_len) # 이미지 크기를 4바이트로 변환

                    robot_id_bytes = struct.pack("<q", self.robot_id)
                    # frame_bytes 생성: 크기(4바이트) + 이미지 데이터 + robot_id
                    frame_bytes = img_len_bytes + img_bytes + robot_id_bytes
                    self.sock.sendto(frame_bytes, self.udp_address)
                    # self.get_logger().info('Frame sent')  # Optional logging
                else:
                    self.get_logger().error('Error encoding frame')
            except cv2.error as e:
                self.get_logger().error(f"OpenCV error: {e}")
            except socket.error as e:
                self.get_logger().error(f"Socket error: {e}")
        else:
            self.get_logger().warn('Could not read frame')   

    def destroy_node(self):
        self.cap.release()
        self.sock.close()
        super().destroy_node()
        self.get_logger().info('Camera and socket closed')

def main(args=None):
    rclpy.init(args=args)
    node = CamUdpPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
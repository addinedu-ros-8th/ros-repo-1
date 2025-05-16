"""
    pinky generate frames from camera and forward them to the server.
"""
import cv2
import socket
# vision related libraries
import struct
# ROS related libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from sensor_msgs.msg import Image
from config.config_node import UserConfig
# Global Initialization
config = UserConfig()



class CamUdpPub(Node):
    def __init__(self):
        super().__init__('cam_udp_pub')
        # self.bridge = CvBridge()     # convert between ROS and OpenCV images
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('camera not opened')
            self.destroy_node()
            return
        self.cmd_map = {"Standby": 0,"Walk": 1,"Patrol": 2}
        self.current_cmd = "Walk"
        # subscriber by main
        self.__sub_cmd = self.create_subscription(String,          # type, x, y 
                                                  '/robot1/schedule', 
                                                  self.schedule_callback, 10)    
        self.__sub_urgent = self.create_subscription(String,       # done
                                                     '/robot1/emergency_msg',  
                                                    self.emergency_callback, 10)
        
        self.default_udp_address = (config.udp_ip, config.udp_port)
        self.current_udp_address = self.default_udp_address

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.timer = self.create_timer(config.frame_rate, self.send_frame)  # 30 FPS
        self.robot_id = 1
        # store the target person's (x, y) for rehabilitation 
        self.temp_x = -1
        self.temp_y = -1

    def schedule_callback(self, msg):
        """
            schedule : Walk, Patrol, Standby
        """
        parts = msg.data.split(',')
        if len(parts) == 3:
            cmd = parts[0]
            try: 
                if cmd == 'Walk':
                    self.current_cmd = 'Walk'
                elif cmd == 'Patrol':
                    self.current_cmd = 'Patrol'
                else:
                    self.current_cmd = 'Standby'

            except ValueError:
                self.get_logger().error(f"잘못된 데이터수신: {msg.data}")  
        else:
            self.get_logger().warn('Could not read frame')

    def send_frame(self):
        ret, frame = self.cap.read()
        if ret:  # Corrected condition
            try:
                resized_frame = cv2.resize(frame, (640, 480))
                _, img_encoded = cv2.imencode(
                    '.jpg', resized_frame, 
                    [int(cv2.IMWRITE_JPEG_QUALITY), 60])  # Encoding parameters
                if _: # Check if encoding was successful
                    img_bytes = img_encoded.tobytes()
                    img_len = len(img_bytes)
                    # self.get_logger().info(f'{img_len}')
                    img_len_bytes = struct.pack(">I", img_len) # 이미지 크기를 4바이트로 변환
                    robot_id_bytes= struct.pack(">B", self.robot_id)
                    # command_bytes = struct.pack(">B", self.cmd_map[self.current_cmd])
                    command_bytes = struct.pack(">B", self.cmd_map[self.current_cmd])
                    # frame_bytes 생성: 크기(4바이트) + 이미지 데이터 + robot_id
                    frame_bytes = img_len_bytes + img_bytes + robot_id_bytes+ command_bytes
                    self.sock.sendto(frame_bytes, self.current_udp_address)

                    if self.current_udp_address != self.default_udp_address:
                        self.sock.sendto(frame_bytes, self.default_udp_address)
                
                else:
                    self.get_logger().error('Error encoding frame')
            except cv2.error as e:
                self.get_logger().error(f"OpenCV error: {e}")
            except socket.error as e:
                self.get_logger().error(f"Socket error: {e}")
        else:
            self.get_logger().warn('Could not read frame')

    def emergency_callback(self, msg):
        if msg.data != 'done':
            received_ip = msg.data
            self.current_udp_address = (received_ip, config.udp_port)
            self.get_logger().info(f'Received emergency IP. Updated current UDP address to: {self.current_udp_address}')
        else:
            self.current_udp_address = self.default_udp_address
            self.get_logger().info(f'Received "done" message. Reverted to default UDP address: {self.current_udp_address}')


        
    
    def destroy_node(self):
        self.cap.release()
        self.sock.close()
        super().destroy_node()
        self.get_logger().info('Camera and socket closed')
        
def main(args=None):
    rclpy.init(args=args, domain_id=1)
    node = CamUdpPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
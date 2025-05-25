
import cv2
import socket
import struct
from picamera2 import Picamera2
from libcamera import Transform
# ROS related libraries
from rclpy.node import Node
from std_msgs.msg import String

class CamNode(Node):
    """
        pinky camera: udp 통신을 통해 ai server에게 frame을 보낸다. 
    """
    def __init__(self, config):
        super().__init__('cam_udp_pub')
        # self.cap = Picamera2()     
        # video_config = self.cap.create_video_configuration(
        #     main={"size": (640, 480)},
        #     transform=Transform(hflip=True, vflip=True)
        # )
        # self.cap.configure(video_config)
        # self.cap.start()
        # self.cap = cv2.VideoCapture('/dev/webcam')
        self.cap = cv2.VideoCapture(8)
        if not self.cap.isOpened():
            self.get_logger().error('camera not opened')
            self.destroy_node()
            return

        self.current_cmd = 'Standby'
        self.cmd_map = {"Standby": 0, "Walk": 1}

        self.__sub_urgent = self.create_subscription(String,       # done
            '/robot1/emergency_msg',self.emergency_callback, 10)
        
        self.udp_ip, self.udp_port = config.udp_ip, config.udp_port
        self.default_udp_address = (self.udp_ip, self.udp_port )
        self.current_udp_address = self.default_udp_address

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.timer = self.create_timer(config.frame_rate, self.send_frame)  # 30 FPS

        self.robot_id = config.robot_id

        self.get_logger().info("CamNode 초기화 완료.") # 추가

    def set_current_cmd(self, cmd):
        self.current_cmd = cmd

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
        # else:
        #     self.get_logger().warn('Could not read frame')

    def emergency_callback(self, msg):
        if msg.data != 'done':
            received_ip = msg.data
            self.current_udp_address = (received_ip, self.udp_port)
            self.get_logger().info(f'Received emergency IP. Updated current UDP address to: {self.current_udp_address}')
        else:
            self.current_udp_address = self.default_udp_address
            self.get_logger().info(f'Received "done" message. Reverted to default UDP address: {self.current_udp_address}')

        

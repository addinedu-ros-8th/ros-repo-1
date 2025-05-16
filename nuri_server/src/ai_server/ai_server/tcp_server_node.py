
import rclpy 
from rclpy.node import Node
import socket 
import struct  # 데이터 크기 처리를 위해
# Global Initialization
from emergency.srv import EmergencyReport  # 서비스 정의 임포트
from config.config_node import UserConfig
config = UserConfig()

import threading 
import time  # 시간 관련 함수 추가
import random  # 랜덤 데이터 생성을 위해

class TcpServerNode(Node):
    def __init__(self):
        super().__init__('tcp_server_node')
        # Initialize the TCP server
        self.server_ip = "192.168.0.43"
        self.server_port = 9999
        self.tcp_server_address = (self.server_ip, self.server_port)
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.logger = self.get_logger()
        
        self.connect_2_main_server()
        # init service network
        self.emergency_service = self.create_service(
            EmergencyReport, 'report_emergency', self.handle_emergency_report)

        self.logger.info('TCP server node ready to receive emergency reports.')

       

    def connect_2_main_server(self):
        try:
            self.tcp_socket.connect(self.tcp_server_address)
            self.logger.info(f'Connected to Main Server on {self.tcp_server_address}')
            
            self.send_client_hello()
        except Exception as e:
            self.logger.error(f'Could not connect to Main Server: {e}')
            self.destroy_node()  # 노드 종료 또는 재시도 로직 구현 필요
            rclpy.shutdown()

    def send_client_hello(self):

        opcode = 0x00  # Opcode (1 Byte)
        server_str = "server"  # Server string (10 Byte: 4 Byte length + 6 Byte data)
        ai_str = "AI"          # AI string (6 Byte: 4 Byte length + 2 Byte data)
        packet_format = ">BH6sH2s"
        packet = struct.pack(
            packet_format,
            opcode,
            len(server_str),
            server_str.encode('utf-8'),
            len(ai_str),
            ai_str.encode('utf-8')
        )
        
        try:
            packet_len = len(packet)
            self.tcp_socket.sendall(struct.pack(">I", packet_len) + packet)
        except Exception as e:
             self.logger.error(f"Error sending CLIENT_HELLO packet: {e}")

    def handle_emergency_report(self, request, response):
        """
        UDP 노드로부터 긴급 상황 보고를 처리하는 서비스 콜백 함수
        """
        robot_id = request.robot_id
        emergency_type = request.emergency_type
        self.logger.info(f'Received emergency report from UDP node: Robot {robot_id} - {emergency_type}')
        
        # Main Server로 긴급 상황 정보 전송
        self.send_emergency_2_main_server(robot_id, emergency_type)

        response.success = True
        response.message = 'Emergency report received and processed by TCP server.'
        return response

    def send_emergency_2_main_server(self, robot_id, emergency_type):
        """
        Main Server로 긴급 상황 정보를 바이너리 형태로 전송하는 함수
        """
        try:
            # emergency_type을 바이트로 인코딩하고 길이를 구함
            emergency_type_bytes = emergency_type.encode('utf-8')
            emergency_type_len = len(emergency_type_bytes)
            opcode = 0x30
            # 바이너리 데이터 생성
            data = struct.pack("<BB", opcode, robot_id) + \
                   struct.pack("<H", emergency_type_len) + emergency_type_bytes
            packet_len = len(data)
            self.tcp_socket.sendall(struct.pack(">I", packet_len) + data)
            self.logger.info(f'Sent emergency to Main Server in binary format: Robot {robot_id} - {emergency_type}')

        except socket.error as e:
            self.logger.error(f'Error sending emergency to Main Server: {e}')


    def destory_node(self):
        self.tcp_socket.close()
        super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    node = TcpServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
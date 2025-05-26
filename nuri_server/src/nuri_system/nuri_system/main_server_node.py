import os
import rclpy
import socket
import configparser
import threading

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory

from nuri_system.client.resident_health_check import ResidentHealthCheck
from nuri_system.robot.robot_handler import RobotHandler
from nuri_system.robot.initial_pose import InitialPose
from nuri_system.handler.socket_handler import SocketHandler
from nuri_system.database.datbase_connection import NuriDatabase

class MainServerNode(Node):
    def __init__(self):
        super().__init__('main_server_node')
        self.server_ready = self.create_publisher(Bool, '/server_ready', 10)

        # config 로드
        config_path = os.path.join(get_package_share_directory('nuri_system'), 'config', 'config.ini')
        self.config = configparser.ConfigParser()
        self.config.read(config_path)

        # DB 연결
        # self.initialize_database()

        robots = [int(x.strip()) for x in self.config['domain']['robot_id'].split(',')]
        self.robot_handler = RobotHandler(robots)

        # TCP 서버 시작
        threading.Thread(target=self.start_server, daemon=True).start()

        self.health = ResidentHealthCheck(self)
        self.check_timer = self.create_timer(5, self.health.health_checker)

        self.get_logger().info('메인 서버 시작')

        msg = Bool()
        msg.data = True
        self.server_ready.publish(msg)


    # def initialize_database(self):
    #     try:
    #         db = self.config['database']
    #         NuriDatabase.initialize(
    #             host=db['host'],
    #             user=db['user'],
    #             password=db['password'],
    #             database=db['database'],
    #         )
    #         self.get_logger().info("[DB] database connection successed")
    #     except Exception as e:
    #         self.get_logger().info(f"[ERROR] database connection failed : {e}")

    def start_server(self):
        host = self.config['main_server']['host']
        port = int(self.config['main_server']['port'])

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((host, port))
            server.listen()
            while True:
                conn, addr = server.accept()
                handler = SocketHandler(conn, addr, self, self.robot_handler)
                handler.start()


def main(args=None):
    rclpy.init(args=args)
    node = MainServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.robot_handler)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
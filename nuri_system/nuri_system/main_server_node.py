import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import socket
import configparser
import threading

from nuri_system.tcp_server.handler.socket_handler import SocketHandler
from nuri_system.tcp_server.database.datbase_connection import NuriDatabase

class MainServerNode(Node):
    def __init__(self):
        super().__init__('main_server_node')

        # config 로드
        self.config = configparser.ConfigParser()
        self.config.read('config.ini')

        # DB 연결
        self.initialize_database()

        # TCP 서버 시작
        threading.Thread(target=self.start_server, daemon=True).start()

    def initialize_database(self):
        try:
            db = self.config['database']
            NuriDatabase.initialize(
                host=db['host'],
                user=db['user'],
                password=db['password'],
                database=db['database'],
            )
            self.get_logger().info("[DB] database connection successed")
        except Exception as e:
            self.get_logger().info(f"[ERROR] database connection failed : {e}")

    def start_server(self):
        host = self.config['main_server']['host']
        port = int(self.config['main_server']['port'])

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((host, port))
            server.listen()
            self.get_logger().info(f"[TCP] 서버 시작: {host}:{port}")
            while True:
                conn, addr = server.accept()
                handler = SocketHandler(conn, addr, self)
                handler.start()


def main(args=None):
    rclpy.init(args=args)
    node = MainServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
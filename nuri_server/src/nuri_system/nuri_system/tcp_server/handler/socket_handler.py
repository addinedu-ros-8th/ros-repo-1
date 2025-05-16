import struct
import threading

from nuri_system.network.packet_reader import PacketReader
from nuri_system.client.client_manager import ClientManager
from nuri_system.handler.client_handler import ClientHandler

class SocketHandler(threading.Thread):
    client_manager = ClientManager()

    def __init__(self, conn, addr, ros_node=None, robot_handler=None):
        super().__init__()
        self.conn = conn
        # self.conn.settimeout(1.0)
        self.addr = addr
        self.ros_node = ros_node
        self.robot_handler = robot_handler

    def run(self):
        try:
            while True:
                try:
                    length_data = self.recv_exact(4)
                    packet_length = struct.unpack('>I', length_data)[0]

                    payload_data = self.recv_exact(packet_length)
                    
                    reader = PacketReader(payload_data)
                    ClientHandler.handle_packet(self, reader, self.ros_node)
                except ConnectionError as e:
                    self.ros_node.get_logger().info(f"[DISCONNECTED] {self.addr}")
                    break
                except Exception as e:
                    self.ros_node.get_logger().info(f"[HANDLE ERROR] {self.addr} -> {e}")
        except Exception as e:
            self.ros_node.get_logger().info(f"[SOCKET ERROR] {self.addr} -> {e}")
        finally:
            self.client_manager.unregister(self)
            self.conn.close()

    def send(self, data: bytes):
        try:
            length = len(data)
            full_data = struct.pack('>I', length) + data
            self.conn.sendall(full_data)
        except Exception as e:
            self.ros_node.get_logger().info(f"[SEND ERROR] {self.addr} -> {e}")

    def recv_exact(self, size):
        """
        size 바이트 만큼 데이터 읽기
        """
        buffer = b''
        while len(buffer) < size:
            chunk = self.conn.recv(size - len(buffer))
            if not chunk:
                raise ConnectionError()
            buffer += chunk
        return buffer
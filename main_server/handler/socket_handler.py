import struct
import threading
from network.packet_reader import PacketReader
from client.client_manager import ClientManager
from handler.client_handler import ClientHandler

class SocketHandler(threading.Thread):
    client_manager = ClientManager()

    def __init__(self, conn, addr):
        super().__init__(daemon=True)
        self.conn = conn
        self.addr = addr

    def run(self):
        try:
            while True:
                try:
                    length_data = self.recv_exact(4)
                    packet_length = struct.unpack('>I', length_data)[0]

                    payload_data = self.recv_exact(packet_length)

                    reader = PacketReader(payload_data)
                    ClientHandler.handle_packet(self, reader)
                except ConnectionError as e:
                    print(f"[DISCONNECTED] {self.addr}")
                    break
                except Exception as e:
                    print(f"[HANDLE ERROR] {self.addr} -> {e}")
        except Exception as e:
            print(f"[SOCKET ERROR] {self.addr} -> {e}")
        finally:
            self.client_manager.unregister(self)
            self.conn.close()

    def send(self, data: bytes):
        try:
            length = len(data)
            full_data = struct.pack('>I', length) + data
            self.conn.sendall(full_data)
        except Exception as e:
            print(f"[SEND ERROR] {self.addr} -> {e}")

    def recv_exact(self, size):
        """
        소켓에서 정확히 size 바이트를 읽을 때까지 recv() 반복
        """
        buffer = b''
        while len(buffer) < size:
            chunk = self.conn.recv(size - len(buffer))
            if not chunk:
                raise ConnectionError()
            buffer += chunk
        return buffer
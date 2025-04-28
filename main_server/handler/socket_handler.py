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
                data = self.conn.recv(4096)
                if not data:
                    print(f"[CLOSED] {self.addr} 연결 종료")
                    self.client_manager.unregister(self)
                    self.conn.close()
                    break

                reader = PacketReader(data)
                ClientHandler.handle_packet(self, reader)
        except Exception as e:
            print(f"[ERROR] {self.addr} -> {e}")

    def send(self, data: bytes):
        try:
            self.conn.sendall(data)
        except Exception as e:
            print(f"[SEND ERROR] {self.addr} -> {e}")
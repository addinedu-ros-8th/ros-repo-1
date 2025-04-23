import threading
from network.packet_reader import PacketReader
from network.packet.client_packet import ClientPacket
from client.client_manager import ClientManager
from handler.robot_handler import Robot
from handler.opcode import Opcode

class ClientHandler(threading.Thread):
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
                opcode = reader.read_command()
                self.handle_packet(opcode, reader)
        except Exception as e:
            print(f"[ERROR] {self.addr} -> {e}")

    def handle_packet(self, opcode, reader):
        if opcode == Opcode.CLIENT_HELLO.value:      # Client Register
            try:
                server = reader.read_string()
                self.client_manager.register(self, server)
            except:
                self.client_manager.register(self)
            finally:
                self.send(ClientPacket.send_hello())
                print(f"[CONNECTED] {self.addr}")

        elif opcode == Opcode.ROBOT_LIST.value:    # Request Robot List
            Robot.fetch_robots(self)
        elif opcode == 0x02:
            text = reader.read_string()
            print(f"[CMD 0x02] Received string: {text}")
        elif opcode == 0x03:
            data = reader.read_bytes()
            print(f"[CMD 0x03] Received {len(data)} bytes of binary data")
        else:
            print(f"[UNKNOWN CMD] 0x{opcode:02X}")

    def send(self, data: bytes):
        try:
            self.conn.sendall(data)
        except Exception as e:
            print(f"[SEND ERROR] {self.addr} -> {e}")
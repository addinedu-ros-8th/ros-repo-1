import threading
from network.PacketReader import PacketReader
from client.ClientManager import ClientManager

class ClientHandler(threading.Thread):
    client_manager = ClientManager()

    def __init__(self, conn, addr):
        super().__init__(daemon=True)
        self.conn = conn
        self.addr = addr

    def run(self):
        print(f"[CONNECTED] {self.addr}")
        self.client_manager.register(self)
        try:
            while True:
                data = self.conn.recv(4096)
                if not data:
                    print(f"[CLOSED] {self.addr} 연결 종료")
                    self.client_manager.unregister(self)
                    self.conn.close()
                    break
                
                reader = PacketReader(data)
                command = reader.read_command()
                self.dispatch_command(command, reader)
        except Exception as e:
            print(f"[ERROR] {self.addr} -> {e}")

    def dispatch_command(self, command, reader):
        if command == 0x01:
            number = reader.read_int()
            number2 = reader.read_string()
            print(f"{number} {number2}")
        elif command == 0x02:
            text = reader.read_string()
            print(f"[CMD 0x02] Received string: {text}")
        elif command == 0x03:
            data = reader.read_bytes()
            print(f"[CMD 0x03] Received {len(data)} bytes of binary data")
        else:
            print(f"[UNKNOWN CMD] 0x{command:02X}")
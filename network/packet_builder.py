import io
import struct

class PacketBuilder:
    def __init__(self):
        self.buf = io.BytesIO()

    def write_command(self, command: int):
        self.buf.write(struct.pack('>H', command))

    def write_int(self, value: int):
        self.buf.write(struct.pack('>i', value))

    def write_string(self, value: str):
        encoded = value.encode('utf-8')
        self.buf.write(struct.pack('>H', len(encoded)))
        self.buf.write(encoded)

    def write_bytes(self, data: bytes):
        self.buf.write(struct.pack('>I', len(data)))
        self.buf.write(data)

    def get_packet(self) -> bytes:
        return self.buf.getvalue()
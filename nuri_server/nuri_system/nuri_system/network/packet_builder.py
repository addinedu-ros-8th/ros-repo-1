import io
import struct

class PacketBuilder:
    def __init__(self):
        self.buf = io.BytesIO()

    def write_opcode(self, opcode: int):
        self.buf.write(struct.pack('>B', opcode))

    def write_byte(self, value: int):
        self.buf.write(struct.pack('>B', value))

    def write_char(self, value: str):
        self.buf.write(struct.pack('>c', value.encode('ascii')))

    def write_short(self, value: int):
        self.buf.write(struct.pack('>H', value))

    def write_int(self, value: int):
        self.buf.write(struct.pack('>i', value))

    def write_string(self, value: str):
        encoded = value.encode('utf-8')
        self.buf.write(struct.pack('>H', len(encoded)))
        self.buf.write(encoded)

    def write_bytes(self, data: bytes):
        self.buf.write(struct.pack('>I', len(data)))
        self.buf.write(data)

    def write_bool(self, value: bool):
        self.buf.write(b'\x01' if value else b'\x00')

    def get_packet(self) -> bytes:
        return self.buf.getvalue()
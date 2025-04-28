import io
import struct

class PacketReader:
    def __init__(self, data: bytes):
        self.buf = io.BytesIO(data)

    def read_opcode(self) -> int:
        return struct.unpack('>H', self.buf.read(2))[0]
    
    def read_status(self) -> int:
        return struct.unpack('>H', self.buf.read(2))[0]

    def read_int(self) -> int:
        return struct.unpack('>i', self.buf.read(4))[0]

    def read_string(self) -> str:
        length = struct.unpack('>H', self.buf.read(2))[0]
        return self.buf.read(length).decode('utf-8')

    def read_bytes(self) -> bytes:
        length = struct.unpack('>I', self.buf.read(4))[0]
        return self.buf.read(length)
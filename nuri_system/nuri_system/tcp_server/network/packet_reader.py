import io
import struct

class PacketReader:
    def __init__(self, data: bytes):
        self.buf = io.BytesIO(data)

    def read_byte(self) -> int:
        return struct.unpack('>B', self.buf.read(1))[0]
    
    def read_char(self) -> str:
        return struct.unpack('>c', self.buf.read(1))[0].decode('ascii')
    
    def read_short(self) -> int:
        return struct.unpack('>H', self.buf.read(2))[0]

    def read_int(self) -> int:
        return struct.unpack('>i', self.buf.read(4))[0]
    
    def read_bool(self) -> bool:
        byte = self.buf.read(1)
        
        return byte != b'\x00'
    
    def read_float(self) -> float:
        return struct.unpack('>f', self.buf.read(4))[0]
    
    def read_double(self):
        return struct.unpack('>Q', self.buf.read(8))[0]

    def read_string(self) -> str:
        length = struct.unpack('>H', self.buf.read(2))[0]
        return self.buf.read(length).decode('utf-8')

    def read_bytes(self) -> bytes:
        length = struct.unpack('>I', self.buf.read(4))[0]
        return self.buf.read(length)
    
    def read_image(self) -> bytes:
        length_bytes = self.buf.read(4)
        if len(length_bytes) < 4:
            raise ValueError("Not enough data to read image length.")
        
        length = struct.unpack('>I', length_bytes)[0]
        image_data = self.buf.read(length)
        if len(image_data) < length:
            raise ValueError("Not enough image data read.")
        
        return image_data
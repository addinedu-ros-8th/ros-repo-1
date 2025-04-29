import io
import struct
from PySide6.QtGui import QPixmap, QImage

class PacketReader:
    def __init__(self, data: bytes):
        self.buf = io.BytesIO(data)

    def read_opcode(self) -> int:
        return struct.unpack('>H', self.buf.read(2))[0]
    
    def read_status(self) -> int:
        return struct.unpack('>B', self.buf.read(1))[0]
    
    def read_char(self) -> str:
        return struct.unpack('>c', self.buf.read(1))[0].decode('ascii')

    def read_int(self) -> int:
        return struct.unpack('>i', self.buf.read(4))[0]

    def read_string(self) -> str:
        length = struct.unpack('>H', self.buf.read(2))[0]
        return self.buf.read(length).decode('utf-8')

    def read_bytes(self) -> bytes:
        length_bytes = self.buf.read(4)
        length = struct.unpack('>I', length_bytes)[0]

        return self.buf.read(length)
    
    def read_image(self) -> QPixmap:
        data = self.read_bytes()  # 이미지 크기 읽고, 데이터 읽기
        pixmap = QPixmap()
        if not pixmap.loadFromData(data, "PNG"):
            raise ValueError("Failed to load image from data.")
        
        image = pixmap.toImage().convertToFormat(QImage.Format.Format_RGB32)
        pixmap = QPixmap.fromImage(image)

        return pixmap
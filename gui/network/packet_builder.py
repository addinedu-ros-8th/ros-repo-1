import io
import struct
from PySide6.QtGui import QPixmap
from PySide6.QtCore import QBuffer, QIODevice

class PacketBuilder:
    def __init__(self):
        self.buf = io.BytesIO()

    def write_opcode(self, command: int):
        self.buf.write(struct.pack('>B', command))

    def write_byte(self, value: int):
        self.buf.write(struct.pack('>B', value))

    def write_short(self, value: int):
        self.buf.write(struct.pack('>H', value))

    def write_int(self, value: int):
        self.buf.write(struct.pack('>i', value))

    def write_bool(self, value: bool):
        self.buf.write(b'\x01' if value else b'\x00')

    def write_float(self, value: float):
        self.buf.write(struct.pack('>f', value))

    def write_char(self, value: str):
        self.buf.write(struct.pack('>c', value.encode('ascii')))

    def write_string(self, value: str):
        encoded = value.encode('utf-8')
        self.buf.write(struct.pack('>H', len(encoded)))
        self.buf.write(encoded)

    def write_bytes(self, data: bytes):
        self.buf.write(struct.pack('>I', len(data)))
        self.buf.write(data)
    
    def write_image(self, pixmap: QPixmap):
        buffer = QBuffer()
        buffer.open(QIODevice.OpenModeFlag.ReadWrite)

        image = pixmap.toImage()
        if not image.save(buffer, "PNG"):
            raise ValueError("Failed to save QPixmap to buffer.")
        
        image_data = buffer.data().data()  # QByteArray → bytes
        self.write_bytes(image_data)

    def get_packet(self) -> bytes:
        return self.buf.getvalue()
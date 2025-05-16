import struct

from network.packet_reader import PacketReader

from PySide6.QtNetwork import QUdpSocket, QHostAddress
from PySide6.QtCore import QObject, Signal

class UDPSocket(QUdpSocket):
    receive_image = Signal(object)

    def __init__(self):
        super().__init__()

        self.udp_socket = QUdpSocket()
        self.udp_socket.bind(QHostAddress.Any, 5000)
        self.udp_socket.readyRead.connect(self.readData)

        self.buffer = bytearray()

    def readData(self):
        while self.udp_socket.hasPendingDatagrams():
            datagram, host, port = self.udp_socket.readDatagram(self.udp_socket.pendingDatagramSize())

            self.buffer += datagram.data()

        while True:
            if len(self.buffer) < 4:
                break  # 아직 길이조차 못 읽음

            packet_size = struct.unpack('>I', self.buffer[:4])[0]

            # if len(self.buffer) < 4 + packet_size:
            #     break  # 전체 이미지 바이트 안 들어왔음

            image_data = self.buffer[4:4 + packet_size]
            robot_id = self.buffer[4+len(image_data):4+len(image_data)+4]
            self.buffer = self.buffer[4 + packet_size + 4:]  # ✅ 버퍼에서 사용한 만큼 제거
            
            reader = PacketReader(image_data)
            self.receive_image.emit(image_data)
import struct

from network.packet_reader import PacketReader

from PySide6.QtNetwork import QUdpSocket, QHostAddress
from PySide6.QtCore import QByteArray, QObject

class UDPSocket(QObject):
    def __init__(self):
        super().__init__()

        self.udp_socket = UDPSocket()
        self.udp_socket.bind(QHostAddress.Any, 9998)
        self.udp_socket.readyRead.connect(self.receive_data)

        self.buffer = bytearray()

    def receive_data(self):
        while self.udp_socket.hasPendingDatagrams():
            datagram, host, port = self.udp_socket.readDatagram(self.udp_socket.pendingDatagramSize())

            self.buffer += datagram.data()

        while True:
            if len(self.buffer) < 4:
                break

            packet_size = struct.unpack('>I', self.buffer[0:4])[0]\
            
            if len(self.buffer) < 4 + packet_size:
                break

            packet_data = self.buffer[4:4 +packet_size]
            reader = PacketReader(packet_data)

            self.buffer = self.buffer[4 + packet_size]
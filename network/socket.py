from network.packet_reader import PacketReader

import struct
from PyQt6.QtNetwork import QTcpSocket, QAbstractSocket
from PyQt6.QtCore import pyqtSignal, QByteArray

class Socket(QTcpSocket):
    receive_data = pyqtSignal(object)

    def __init__(self):
        super().__init__()

        self.socket = QTcpSocket()
        self.socket.readyRead.connect(self.readData)
        # self.socket.errorOccurred.connect(self.handleError)

    def connectToServer(self, host, port):
        self.socket.connectToHost(host, port)

        if not self.socket.waitForConnected(5000):
            return False
        
        return True
    
    def readData(self):
        if not hasattr(self, 'buffer'):
            self.buffer = QByteArray()

        self.buffer += self.socket.readAll()

        while True:
            if self.buffer.size() < 4:
                break

            length_bytes = self.buffer.mid(0, 4)
            packet_size = struct.unpack('>I', bytes(length_bytes))[0]

            if self.buffer.size() < 4 + packet_size:
                # 아직 패킷 전체가 도착 안 했음
                break

            # 패킷 데이터 추출
            packet_data = self.buffer.mid(4, packet_size)

            # PacketReader에 넘기기
            reader = PacketReader(bytes(packet_data))
            self.receive_data.emit(reader)

            # 읽은 데이터 버퍼에서 제거
            self.buffer = self.buffer.mid(4 + packet_size)

    def sendData(self, data):
        if self.socket.state() == QAbstractSocket.SocketState.ConnectedState:
            packet_length = len(data)
            full_data = struct.pack('>I', packet_length) + data
            self.socket.write(full_data)
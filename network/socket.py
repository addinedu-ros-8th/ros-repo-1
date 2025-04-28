from network.packet_reader import PacketReader

from PyQt6.QtNetwork import QTcpSocket, QAbstractSocket
from PyQt6.QtCore import pyqtSignal

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
        while self.socket.bytesAvailable() > 0:
            data = self.socket.readAll()

            reader = PacketReader(data)

            self.receive_data.emit(reader)
            
    def sendData(self, data):
        if self.socket.state() == QAbstractSocket.SocketState.ConnectedState:
            self.socket.write(data)
from handler.opcode import Opcode

from PyQt6.QtCore import pyqtSignal

class PacketHandler():
    def handle_packet(socket, widgets, packet):

        opcode = packet.read_opcode()

        if opcode == Opcode.ROBOT_LIST.value:
            pass
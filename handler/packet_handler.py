from handler.opcode import Opcode
from network.packet import Packet

class PacketHandler():
    def handle_packet(socket, widgets, packet):
        opcode = packet.read_command()

        if opcode == Opcode.CLIENT_HELLO.value:
            socket.sendData(Packet.robot_list())
        elif opcode == Opcode.ROBOT_LIST.value:
            size = packet.read_int()

            widgets.bot_list.clear()
            for idx in range(size):
                widgets.bot_list.addItem(packet.read_string())

            widgets.stackedWidget.setCurrentWidget(widgets.home)
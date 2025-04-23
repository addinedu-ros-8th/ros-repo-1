from handler.opcode import Opcode
from network.packet import Packet

class PacketHandler():
    def handle_packet(socket, widgets, packet):
        opcode = packet.read_command()

        if opcode == Opcode.ROBOT_LIST.value:
            size = packet.read_int()

            widgets.bot_list.clear()
            for _ in range(size):
                widgets.bot_list.addItem(packet.read_string())

            widgets.bot_list.setCurrentIndex(-1)
            widgets.movie_label.deleteLater()
            widgets.stackedWidget.setCurrentWidget(widgets.home)
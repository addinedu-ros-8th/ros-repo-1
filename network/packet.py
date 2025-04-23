from network.packet_builder import PacketBuilder
from handler.opcode import Opcode

class Packet:
    def client_hello():
        packet = PacketBuilder()

        packet.write_command(Opcode.CLIENT_HELLO.value)

        return packet.get_packet()
    
    def robot_list():
        packet = PacketBuilder()

        packet.write_command(Opcode.ROBOT_LIST.value)

        return packet.get_packet()
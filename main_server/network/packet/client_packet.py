from handler.opcode import Opcode
from network.packet_builder import PacketBuilder

class ClientPacket:

    def send_hello():
        packet = PacketBuilder()

        packet.write_opcode(Opcode.CLIENT_HELLO.value)
        packet.write_int(1)

        return packet.get_packet()
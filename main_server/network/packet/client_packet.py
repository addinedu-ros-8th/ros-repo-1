from handler.opcode import Opcode
from network.packet_builder import PacketBuilder

class ClientPacket:

    def send_hello():
        packet = PacketBuilder()

        packet.write_opcode(Opcode.CLIENT_HELLO.value)
        packet.write_status(0x00)

        return packet.get_packet()
    
    def send_resident_list(status, result):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.RESIDENT_LIST.value)
        packet.write_status(status)

        if status == 0x00:
            packet.write_int(len(result))
            for row in result:
                packet.write_string(row[0])
                packet.write_string(row[1].strftime('%y-%m-%d'))

        return packet.get_packet()
    
    def send_resident_info_result(result):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.SEND_RESIDENT_INFO.value)
        packet.write_status(result)

        return packet.get_packet()
from network.packet_builder import PacketBuilder
from handler.opcode import Opcode

class Packet:
    def client_hello():
        packet = PacketBuilder()

        packet.write_opcode(Opcode.CLIENT_HELLO.value)

        return packet.get_packet()
    
    def request_resident_list():
        packet = PacketBuilder()

        packet.write_opcode(Opcode.RESIDENT_LIST.value)

        return packet.get_packet()
    
    def send_resident_info(resident_name, birthday, sex, room_number, bed_number):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.SEND_RESIDENT_INFO.value)
        packet.write_string(resident_name)
        packet.write_string(birthday)
        packet.write_char(sex)
        packet.write_int(room_number)
        packet.write_int(bed_number)

        return packet.get_packet()
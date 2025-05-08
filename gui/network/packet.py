from network.packet_builder import PacketBuilder
from handler.opcode import Opcode

class Packet:
    def client_hello():
        packet = PacketBuilder()

        packet.write_opcode(Opcode.CLIENT_HELLO.value)
        packet.write_string("client")

        return packet.get_packet()
    
    def request_resident_list(name=None, check=None):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.RESIDENT_LIST.value)
        if name is None:
            name = ""
        packet.write_string(name)
        packet.write_bool(check)

        return packet.get_packet()
    
    def send_resident_info(resident_name, birthday, sex, room_number, bed_number, face):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.SEND_RESIDENT_INFO.value)
        packet.write_string(resident_name)
        packet.write_string(birthday)
        packet.write_char(sex)
        packet.write_short(room_number)
        packet.write_short(bed_number)
        packet.write_image(face)

        return packet.get_packet()
    
    def request_resident_info(resident_name, birthday):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.REQUEST_RESIDENT_INFO.value)
        packet.write_string(resident_name)
        packet.write_string(birthday)

        return packet.get_packet()
    
    def request_discharge(name, birthday):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.REQUEST_DISCHARGE.value)
        packet.write_string(name)
        packet.write_string(birthday)

        return packet.get_packet()
    
    def update_resident_info(resident_name, birthday, sex, room_number, bed_number):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.UPDATE_RESIDENT_INFO.value)
        packet.write_string(resident_name)
        packet.write_string(birthday)
        packet.write_char(sex)
        packet.write_int(room_number)
        packet.write_int(bed_number)

        return packet.get_packet()
    
    def delete_resident_info(resident_name, birthday):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.DELETE_RESIDENT.value)
        packet.write_string(resident_name)
        packet.write_string(birthday)

        return packet.get_packet()
    
    def request_log_category():
        packet = PacketBuilder()

        packet.write_opcode(Opcode.LOG_CATEGORY.value)

        return packet.get_packet()
    
    def request_video(ipaddress):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.REQUEST_VIDEO.value)
        packet.write_string(ipaddress)

        return packet.get_packet()
    
    def request_robot_list():
        packet = PacketBuilder()

        packet.write_opcode(Opcode.ROBOT_LIST.value)

        return packet.get_packet()
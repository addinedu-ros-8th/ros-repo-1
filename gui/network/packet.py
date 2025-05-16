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
    
    def request_log_list(start, end, event_type, robot, keyword):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.LOG_SEARCH.value)
        packet.write_string(start)
        packet.write_string(end)
        packet.write_string(event_type)
        packet.write_byte(robot)
        packet.write_string(keyword)

        return packet.get_packet()
    
    def request_video(ipaddress, robot_id):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.REQUEST_VIDEO.value)
        packet.write_string(ipaddress)
        packet.write_byte(robot_id)

        return packet.get_packet()
    
    def request_robot_list():
        packet = PacketBuilder()

        packet.write_opcode(Opcode.ROBOT_LIST.value)

        return packet.get_packet()
    
    def request_patrol_schedule():
        packet = PacketBuilder()

        packet.write_opcode(Opcode.PATROL_LIST.value)

        return packet.get_packet()
    
    def regist_partrol(time):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.PATROL_REGIST.value)
        packet.write_string(time)

        return packet.get_packet()
    
    def unregist_patrol(time):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.PATROL_UNREGIST.value)
        packet.write_string(time)

        return packet.get_packet()
    
    def request_resident_name_list():
        packet = PacketBuilder()

        packet.write_opcode(Opcode.RESIDENT_NAME_LIST.value)

        return packet.get_packet()
    
    def request_walk_schedule():
        packet = PacketBuilder()

        packet.write_opcode(Opcode.WALK_LIST.value)
        
        return packet.get_packet()
    
    def regist_walk(name, time):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.WALK_REGIST.value)
        packet.write_string(name)
        packet.write_string(time)

        return packet.get_packet()
    
    def unregist_walk(name, time):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.WALK_UNREGIST.value)
        packet.write_string(name)
        packet.write_string(time)

        return packet.get_packet()
    
    def send_goal_pose(x, y):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.GOAL_POSE.value)
        packet.write_float(x)
        packet.write_float(y)
        packet.write_byte(1)

        return packet.get_packet()
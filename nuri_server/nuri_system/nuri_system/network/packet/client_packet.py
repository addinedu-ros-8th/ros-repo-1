from nuri_system.handler.opcode import Opcode
from nuri_system.network.packet_builder import PacketBuilder

class ClientPacket:

    def send_hello():
        packet = PacketBuilder()

        packet.write_opcode(Opcode.CLIENT_HELLO.value)
        packet.write_byte(0x00)

        return packet.get_packet()
    
    def send_resident_list(status, result):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.RESIDENT_LIST.value)
        packet.write_byte(status)

        if status == 0x00:
            packet.write_short(len(result))
            for row in result:
                packet.write_string(row[0])
                packet.write_string(row[1].strftime('%Y-%m-%d'))
                packet.write_char(row[2])

        return packet.get_packet()
    
    def send_resident_info_result(result):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.SEND_RESIDENT_INFO.value)
        packet.write_byte(result)

        return packet.get_packet()
    
    def send_resident_info(status, result):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.REQUEST_RESIDENT_INFO.value)
        packet.write_byte(status)
        if status == 0x00:
            packet.write_string(result[0])
            packet.write_string(result[2].strftime('%Y-%m-%d'))
            packet.write_char(result[1])
            packet.write_bytes(result[3])   # Images

        return packet.get_packet()
    
    def discharge_result(status, name):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.REQUEST_DISCHARGE.value)
        packet.write_byte(status)
        packet.write_string(name)

        return packet.get_packet()
    
    def update_resident_info_result(status):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.UPDATE_RESIDENT_INFO.value)
        packet.write_byte(status)

        return packet.get_packet()
    
    def delete_resident_info_result(status):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.DELETE_RESIDENT.value)
        packet.write_byte(status)

        return packet.get_packet()
    
    def send_log_category(status, types, names):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.LOG_CATEGORY.value)
        packet.write_byte(status)
        if status == 0x00:
            packet.write_byte(len(types))
            for type in types:
                packet.write_string(type)

            packet.write_byte(len(names))
            for name in names:
                packet.write_string(name)

        return packet.get_packet()
    
    def send_detection(robot_id, type):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.DETECTION.value)
        packet.write_byte(robot_id)
        packet.write_string(type)

        return packet.get_packet()
    
    def send_robot_list(robots):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.ROBOT_LIST)
        size = len(robots)
        packet.write_byte(size)

        if size != 0:
            for robot in robots:
                packet.write_byte(robot.id)
                packet.write_string(robot.status)
                packet.write_byte(robot.battery)
                packet.write_bool(robot.online)

        return packet.get_packet()
    
    def test2(status):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.RESIDENT_HEALTH_INFO.value)
        packet.write_short(status)

        return packet.get_packet()
from nuri_system.handler.opcode import Opcode
from nuri_system.network.packet_builder import PacketBuilder

class ClientPacket:

    def send_hello():
        packet = PacketBuilder()

        packet.write_opcode(Opcode.CLIENT_HELLO.value)
        packet.write_ubyte(0x00)

        return packet.get_packet()
    
    def send_resident_list(status, result):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.RESIDENT_LIST.value)
        packet.write_ubyte(status)

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
        packet.write_ubyte(result)

        return packet.get_packet()
    
    def send_resident_info(status, result):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.REQUEST_RESIDENT_INFO.value)
        packet.write_ubyte(status)
        if status == 0x00:
            packet.write_string(result[0])
            packet.write_string(result[2].strftime('%Y-%m-%d'))
            packet.write_char(result[1])
            packet.write_bytes(result[3])   # Images

        return packet.get_packet()
    
    def discharge_result(status, name):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.REQUEST_DISCHARGE.value)
        packet.write_ubyte(status)
        packet.write_string(name)

        return packet.get_packet()
    
    def update_resident_info_result(status):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.UPDATE_RESIDENT_INFO.value)
        packet.write_ubyte(status)

        return packet.get_packet()
    
    def delete_resident_info_result(status):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.DELETE_RESIDENT.value)
        packet.write_ubyte(status)

        return packet.get_packet()
    
    def send_log_category(status, types, names):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.LOG_CATEGORY.value)
        packet.write_ubyte(status)
        if status == 0x00:
            packet.write_ubyte(len(types))
            for type in types:
                packet.write_string(type)

            packet.write_ubyte(len(names))
            for name in names:
                packet.write_string(name)

        return packet.get_packet()
    
    def send_log_list(status, result):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.LOG_SEARCH.value)
        packet.write_ubyte(status)
        if result is not None:
            packet.write_short(len(result))
            for item in result:
                packet.write_string(item[0])
                packet.write_ubyte(item[1])
                packet.write_string(item[2])
                packet.write_string(item[3].strftime('%F %T'))

        return packet.get_packet()
    
    def send_detection(robot_id, type):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.DETECTION.value)
        packet.write_ubyte(robot_id)
        packet.write_string(type)

        return packet.get_packet()
    
    def send_robot_list(robots):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.ROBOT_LIST.value)
        size = len(robots)
        packet.write_ubyte(size)

        if size != 0:
            for robot in robots:
                packet.write_ubyte(robot.id)
                packet.write_string(robot.status)
                packet.write_ubyte(robot.battery)
                packet.write_bool(robot.online)

        return packet.get_packet()
    
    def send_robot_location(robots):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.ROBOT_LOCATION.value)
        packet.write_short(len(robots))
        for robot in robots:
            packet.write_float(robot.x)
            packet.write_float(robot.y)
            packet.write_float(robot.q_x)
            packet.write_float(robot.q_y)
            packet.write_float(robot.q_z)
            packet.write_float(robot.q_w)

        return packet.get_packet()
    
    def send_patrol_schedule(status, list=None):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.PATROL_LIST.value)
        packet.write_ubyte(status)
        if status == 0x00:
            packet.write_ubyte(len(list))
            for row in list:
                packet.write_string(str(row[0]))

        return packet.get_packet()
    
    def regist_patrol_result(status):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.PATROL_REGIST.value)
        packet.write_ubyte(status)

        return packet.get_packet()

    def unregist_patrol_result(status):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.PATROL_UNREGIST.value)
        packet.write_ubyte(status)

        return packet.get_packet()
    
    def patrol_mode():
        packet = PacketBuilder()

        packet.write_opcode(Opcode.PATROL_MODE.value)
        packet.write_ubyte(0x01)

        return packet.get_packet()
    
    def send_resident_name_list(status, result):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.RESIDENT_NAME_LIST.value)
        packet.write_ubyte(status)

        if status == 0x00:
            packet.write_short(len(result))
            for name in result:
                packet.write_string(name[0])

        return packet.get_packet()
    
    def regist_walk_result(status):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.WALK_REGIST.value)
        packet.write_ubyte(status)

        return packet.get_packet()
    
    def send_walk_schedule(status, result):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.WALK_LIST.value)
        packet.write_ubyte(status)

        if status == 0x00:
            packet.write_short(len(result))
            for walk in result:
                packet.write_string(walk[0])
                packet.write_string(str(walk[1]))

        return packet.get_packet()

    
    def unregist_walk_result(status):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.WALK_UNREGIST.value)
        packet.write_ubyte(status)

        return packet.get_packet()
    
    def test2(status):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.RESIDENT_HEALTH_INFO.value)
        packet.write_short(status)

        return packet.get_packet()

    def send_map(map_info, img):
        packet = PacketBuilder()

        packet.write_opcode(Opcode.MAP.value)
        for info in map_info.values():
            packet.write_float(info)
            
        packet.write_bytes(img)

        return packet.get_packet()
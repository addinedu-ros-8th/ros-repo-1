from network.packet_builder import PacketBuilder
from handler.opcode import Opcode

class RobotPacket:
    def robot_list(data):
        builder = PacketBuilder()

        builder.write_command(Opcode.ROBOT_LIST.value)
        builder.write_int(len(data))
        for idx in range(len(data)):
            builder.write_string(data[idx][1])

        return builder.get_packet()
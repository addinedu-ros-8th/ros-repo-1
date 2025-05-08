import datetime

from std_msgs.msg import String

from nuri_system.tcp_server.database.datbase_connection import NuriDatabase
from nuri_system.tcp_server.handler.opcode import Opcode
from nuri_system.tcp_server.network.packet.client_packet import ClientPacket

class ClientHandler():

    def handle_packet(handler, reader, node=None):
        opcode = reader.read_byte()

        # print(f"[OPCODE] {Opcode(opcode).name}")
        # node.get_logger().info(f"{Opcode(opcode).name}")

        if opcode == Opcode.CLIENT_HELLO.value:     # Client Register
            ClientHandler.client_hello(handler, reader, node)
        if opcode == Opcode.RESIDENT_LIST.value:
            ClientHandler.fetch_resident_list(handler, reader)
        elif opcode == Opcode.SEND_RESIDENT_INFO.value:
            ClientHandler.add_new_resident(handler, reader)
        elif opcode == Opcode.REQUEST_RESIDENT_INFO.value:
            ClientHandler.fetch_resident_info(handler, reader)
        elif opcode == Opcode.REQUEST_DISCHARGE.value:
            ClientHandler.discharge_resident(handler, reader)
        elif opcode == Opcode.UPDATE_RESIDENT_INFO.value:
            ClientHandler.update_resident_info(handler, reader)
        elif opcode == Opcode.DELETE_RESIDENT.value:
            ClientHandler.delete_resident_info(handler, reader)
        elif opcode == Opcode.RESIDENT_HEALTH_INFO.value:
            ClientHandler.update_health_info(handler, reader, node)
        elif opcode == Opcode.LOG_CATEGORY.value:
            ClientHandler.fetch_log_category(handler, reader, node)
        elif opcode == Opcode.DETECTION.value:
            ClientHandler.detection(handler, reader, node)
        elif opcode == 0x99:
            ClientHandler.test(handler, reader, node)

    @staticmethod
    def client_hello(handler, reader, node):
        type = reader.read_string()
        identifier = None

        if type == "server":
            identifier = reader.read_string()
        elif type == "device":
            identifier = reader.read_int()

        handler.client_manager.register(handler, type, identifier)
        node.get_logger().info(f"[CONNECTED] type : {type}, {handler.addr}")

        handler.send(ClientPacket.send_hello())

    @staticmethod
    def fetch_resident_list(handler, reader):
        conn = NuriDatabase.get_instance()

        status = 0x00

        name = reader.read_string()
        check = reader.read_bool()
        result = None
        params = ()

        try:
            query = "SELECT name, birthday, sex FROM residents"
            if name != "":
                query += " WHERE name like %s"
                params = (f"%{name}%",)

            if not check:
                join = " WHERE" if name == "" else " AND"
                query += join + " discharge_date is null"

            result = conn.fetch_all(query, params)
        except:
            status = 0xFF

        handler.send(ClientPacket.send_resident_list(status, result))

    @staticmethod
    def add_new_resident(handler, reader):
        conn = NuriDatabase.get_instance()
        name = reader.read_string()
        birthday = reader.read_string()
        sex = reader.read_char()
        room_number = reader.read_int()
        bed_number = reader.read_int()
        face = reader.read_image()

        status = 0x00

        try:
            query = "SELECT * FROM residents WHERE name = %s AND birthday = %s"
            result = conn.fetch_one(query, (name, birthday))

            if result:
                status = 0x01
            else:
                query = "INSERT INTO residents(name, birthday, sex) values(%s, %s, %s)"
                user_id = conn.execute_query(query, (name, birthday, sex))

                query = "INSERT INTO faces(user_id, face_image) values(%s, %s)"
                conn.execute_query(query, (user_id, face))
        except Exception as e:
            status = 0xFF
            conn.rollback()

        handler.send(ClientPacket.send_resident_info_result(status))

    @staticmethod
    def fetch_resident_info(handler, reader):
        conn = NuriDatabase.get_instance()

        name = reader.read_string()
        birthday = reader.read_string()

        status = 0x00
        result = None

        try:
            query = """
                    SELECT r.name, r.sex, r.birthday, f.face_image
                    FROM residents r, faces f
                    WHERE r.name = %s and r.birthday = %s and r.id = f.resident_id
                    """
            result = conn.fetch_one(query, (name, birthday))

        except Exception as e:
            status = 0xFF

        handler.send(ClientPacket.send_resident_info(status, result))

    @staticmethod
    def discharge_resident(handler, reader):
        conn = NuriDatabase.get_instance()
        name = reader.read_string()
        birthday = reader.read_string()
        date = datetime.datetime.now()

        status = 0x00

        try:
            query = "UPDATE residents SET discharge_date = %s WHERE name = %s and birthday = %s"
            conn.execute_query(query, (date, name, birthday))
        except:
            status = 0xFF
            conn.rollback()

        handler.send(ClientPacket.discharge_result(status, name))

    @staticmethod
    def update_resident_info(handler, reader):
        conn = NuriDatabase.get_instance()
        name = reader.read_string()
        birthday = reader.read_string()
        sex = reader.read_char()
        bed = reader.read_int()
        room = reader.read_int()

        status = 0x00

        try:
            query = "UPDATE residents SET sex = %s WHERE name = %s and birthday = %s"
            conn.execute_query(query, (sex, name, birthday))
        except:
            status = 0xFF
            conn.rollback()

        handler.send(ClientPacket.update_resident_info_result(status))

    @staticmethod
    def delete_resident_info(handler, reader):
        conn = NuriDatabase.get_instance()
        name = reader.read_string()
        birthday = reader.read_string()

        status = 0x00
        
        try:
            query = "DELETE FROM residents WHERE name = %s and birthday = %s"
            conn.execute_query(query, (name, birthday))
        except:
            status = 0xFF
            conn.rollback()

        handler.send(ClientPacket.delete_resident_info_result(status))

    @staticmethod
    def update_health_info(handler, reader, node):
        conn = NuriDatabase.get_instance()
        resident_id = reader.read_byte()
        heart_rate = reader.read_int()
        oxygen = reader.read_int()
        temperature = reader.read_float()

        node.get_logger().info(f"{resident_id} {heart_rate} {oxygen} {temperature}")

        # handler.send(ClientPacket.test2(200))

    @staticmethod
    def fetch_log_category(handler, reader, node):
        conn = NuriDatabase.get_instance()

        status = 0x00

        try:
            query = "SELECT type FROM log_event"
            type = conn.fetch_all(query)
            type = [row[0] for row in type]

            query = "SELECT name FROM robot"
            name = conn.fetch_all(query)
            name = [row[0] for row in name]
        except:
            conn.rollback()
            status = 0xFF

        handler.send(ClientPacket.send_log_category(status, type, name))

    @staticmethod
    def detection(handler, reader, node):
        conn = NuriDatabase.get_instance()

        robot_id = reader.read_byte()
        type = reader.read_string()

        packet = ClientPacket.send_detection(robot_id, type)

        handler.client_manager.broadcast(packet)

    @staticmethod
    def test(handler, reader, node):
        def cb(msg):
            handler.send(ClientPacket.test(msg.data))

            def destroy():
                node.destroy_subscription(sub)

            node.create_timer(0.01, destroy)

        index = reader.read_short() + 1
        sub = node.create_subscription(String, f'/robot{index}/chatter', cb, 10)
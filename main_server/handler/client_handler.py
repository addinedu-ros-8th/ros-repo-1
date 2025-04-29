import datetime

from database.datbase_connection import NuriDatabase
from handler.opcode import Opcode
from network.packet.client_packet import ClientPacket

class ClientHandler():

    def handle_packet(handler, reader):
        opcode = reader.read_opcode()

        print(f"[OPCODE] {Opcode(opcode).name}")

        if opcode == Opcode.CLIENT_HELLO.value:     # Client Register
            try:
                server = reader.read_string()
                handler.client_manager.register(handler, server)
            except:
                handler.client_manager.register(handler)
            finally:
                handler.send(ClientPacket.send_hello())
                print(f"[CONNECTED] {handler.addr}")
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
                    WHERE r.name = %s and r.birthday = %s and r.id = f.user_id
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

        handler.send(ClientPacket.send_discharge_result(status, name))

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

        handler.send(ClientPacket.send_update_resident_info_result(status))
        
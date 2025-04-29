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
            ClientHandler.fetch_resident_list(handler)
        elif opcode == Opcode.SEND_RESIDENT_INFO.value:
            ClientHandler.add_new_resident(handler, reader)
        elif opcode == Opcode.REQUEST_RESIDENT_INFO.value:
            ClientHandler.fetch_resident_info(handler, reader)


    @staticmethod
    def fetch_resident_list(handler):
        conn = NuriDatabase.get_instance()

        status = 0x00

        try:
            query = "SELECT name, birthday bed_id FROM residents"
            result = conn.fetch_all(query)
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

        with open('face.png', 'wb') as f:
            f.write(face)

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
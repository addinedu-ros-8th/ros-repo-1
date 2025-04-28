from database.datbase_connection import NuriDatabase
from handler.opcode import Opcode
from network.packet.client_packet import ClientPacket

class ClientHandler():

    def handle_packet(handler, reader):
        opcode = reader.read_opcode()

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


    @staticmethod
    def fetch_resident_list(handler, reader):
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

        status = 0x00

        try:
            query = "SELECT * FROM residents WHERE name = %s AND birthday = %s"
            result = conn.fetch_one(query, (name, birthday))

            if result:
                status = 0x01
            else:
                query = "INSERT INTO residents(name, birthday, sex) values(%s, %s, %s)"
                conn.execute_query(query, (name, birthday, sex))
        except Exception as e:
            status = 0xFF
            print(e)

        handler.send(ClientPacket.send_resident_info_result(status))
from database.datbase_connection import NuriDatabase
from network.packet.robot_packet import RobotPacket

class Robot:
    def fetch_robots(handler):
        try:
            query = "SELECT * FROM robot"

            conn = NuriDatabase.get_instance()
            result = conn.fetch_all(query)
            if result:
                handler.send(RobotPacket.robot_list(result))
        except Exception as e:
            print("[ERROR] Failed to fetch robot list from database", e)
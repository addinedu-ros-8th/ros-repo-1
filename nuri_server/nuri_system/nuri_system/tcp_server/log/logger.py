from nuri_system.tcp_server.database.datbase_connection import NuriDatabase

class Logger():
    def write(type, message):
        conn = NuriDatabase.get_instance()

        try:
            query = "INSERT INTO logs(e)"
            conn.execute_query(query, (type, message))
        except:
            conn.rollback()
        
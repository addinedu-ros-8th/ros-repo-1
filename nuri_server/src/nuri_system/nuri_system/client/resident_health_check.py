from datetime import datetime

from nuri_system.database.datbase_connection import NuriDatabase

class ResidentHealthCheck():
    def __init__(self, node):
        self.node = node
        self.last_time = datetime.now()

    def health_checker(self):
        conn = NuriDatabase()

        result = None
        try:
            query = """
                SELECT user_id, ROUND(AVG(temperature), 2) AS temperature
                FROM health_info
                GROUP BY user_id
                HAVING temperature >= 36.5
            """
            result = conn.fetch_all(query)
        except:
            pass

        if (datetime.now() - self.last_time).total_seconds() >= 60:
            self.last_time = datetime.now()
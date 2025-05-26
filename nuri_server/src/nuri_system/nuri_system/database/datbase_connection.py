import os
import mysql.connector
import configparser

from mysql.connector import pooling
from ament_index_python.packages import get_package_share_directory

class NuriDatabase:
    def __init__(self):
        config_path = os.path.join(
            get_package_share_directory('nuri_system'),
            'config',
            'config.ini'
        )
        config = configparser.ConfigParser()
        config.read(config_path)

        db_config = {
            'host': config['database']['host'],
            'user': config['database']['user'],
            'password': config['database']['password'],
            'database': config['database']['database']
        }

        self.pool = mysql.connector.pooling.MySQLConnectionPool(
            pool_name="nuri_pool",
            pool_size=5,
            pool_reset_session=True,
            **db_config
        )

    def execute_query(self, query, param=None):
        conn = self.pool.get_connection()
        cursor = conn.cursor()
        try:
            cursor.execute(query, param)
            conn.commit()
            return cursor.lastrowid
        except Exception as e:
            print(f"[DB ERROR] execute_query failed: {e}")
            conn.rollback()
            self.execute_query(query, param)
        finally:
            cursor.close()
            conn.close()


    def fetch_one(self, query, param=None):
        conn = self.pool.get_connection()
        cursor = conn.cursor(dictionary=False)
        try:
            cursor.execute(query, param)
            return cursor.fetchone()
        except Exception as e:
            print(f"[DB ERROR] execute_query_fetchone failed: {e}")
            self.fetch_one(query, param)
            return None
        finally:
            cursor.close()
            conn.close()

    def fetch_all(self, query, param=None):
        conn = self.pool.get_connection()
        cursor = conn.cursor(dictionary=False)
        try:
            cursor.execute(query, param)
            return cursor.fetchall()
        except Exception as e:
            print(f"[DB ERROR] execute_query_fetchall failed: {e}")
            self.fetch_all(query, param)
            return None
        finally:
            cursor.close()
            conn.close()


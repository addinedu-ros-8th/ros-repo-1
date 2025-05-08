# db.py
import mysql.connector
from threading import Lock

class NuriDatabase:
    _instance = None
    _lock = Lock()

    def __init__(self, host, user, password, database):
        try:
            self.conn = mysql.connector.connect(
                host=host,
                user=user,
                password=password,
                database=database,
                autocommit=True
            )
            print("[SUCCESS] Databse initialized")
        except:
            print("[ERROR] Failed to connect to the database")

    @classmethod
    def initialize(cls, host, user, password, database):
        with cls._lock:
            if cls._instance is None or not cls._instance.conn.is_connected():
                cls._instance = cls(host, user, password, database)

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            raise Exception("Database not initialized. Call initialize() first.")
        return cls._instance

    # 이하 동일
    def execute_query(self, query, param=None):
        cursor = self.conn.cursor()
        cursor.execute(query, param)
        last_id = cursor.lastrowid
        cursor.close()
        return last_id

    def execute_many(self, query, param=None):
        cursor = self.conn.cursor(prepared=True)
        cursor.executemany(query, param)
        cursor.close()
        return True

    def fetch_all(self, query, params=None):
        cursor = self.conn.cursor(buffered=True)
        try:
            cursor.execute(query, params)
            return cursor.fetchall()
        finally:
            cursor.close()

    def fetch_one(self, query, params=None):
        cursor = self.conn.cursor(buffered=True)
        try:
            cursor.execute(query, params)
            return cursor.fetchone()
        finally:
            cursor.close()

    def commit(self):
        self.conn.commit()

    def rollback(self):
        self.conn.rollback()

    def dispose(self):
        if self.conn.is_connected():
            self.conn.close()
            print("disconnect db...")

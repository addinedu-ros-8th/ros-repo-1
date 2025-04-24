import socket
import configparser

from handler.client_handler import ClientHandler
from database.datbase_connection import NuriDatabase

HOST = '0.0.0.0'
PORT = 9999

def start_server():
    print(f"[SERVER] Starting TCP server on {HOST}:{PORT}")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((HOST, PORT))
        server.listen()
        while True:
            conn, addr = server.accept()
            handler = ClientHandler(conn, addr)
            handler.start()

def initialize_database():
    config = configparser.ConfigParser()
    config.read('config.ini')

    host = config['database']['host']
    user = config['database']['user']
    password = config['database']['password']
    database = config['database']['database']

    NuriDatabase.initialize(
        host=host,
        user=user,
        password=password,
        database=database,
    )

if __name__ == "__main__":
    initialize_database()

    start_server()
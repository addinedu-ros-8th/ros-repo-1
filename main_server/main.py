import socket
import configparser

from handler.socket_handler import SocketHandler
from database.datbase_connection import NuriDatabase

def start_server(config):
    host = config['main_server']['host']
    port = int(config['main_server']['port'])

    print(f"[SERVER] Starting TCP server on {host}:{port}")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((host, port))
        server.listen()
        while True:
            conn, addr = server.accept()
            handler = SocketHandler(conn, addr)
            handler.start()

def initialize_database(config):
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
    config = configparser.ConfigParser()
    config.read('config.ini')

    initialize_database(config)

    start_server(config)
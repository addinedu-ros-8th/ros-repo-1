from handler.ClientHandler import ClientHandler
import socket

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


if __name__ == "__main__":
    start_server()
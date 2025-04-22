import socket
from network.PacketBuilder import PacketBuilder

def main():
    builder = PacketBuilder()
    builder.write_command(0x01)
    builder.write_int(40)
    builder.write_string("hello")
    packet = builder.get_packet()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect(('localhost', 9999))
        sock.sendall(packet)

if __name__ == "__main__":
    main()
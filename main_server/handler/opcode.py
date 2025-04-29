from enum import Enum

class Opcode(Enum):
    CLIENT_HELLO = 0x00
    RESIDENT_LIST = 0x01
    SEND_RESIDENT_INFO = 0x02
    REQUEST_RESIDENT_INFO = 0x03
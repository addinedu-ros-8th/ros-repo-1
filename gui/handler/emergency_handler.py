import numpy as np
import struct
import cv2

from PySide6.QtGui import QPixmap

class EmergencyHandler():
    def handle_packet(parent, packet):
        pixmap = QPixmap()
        if not pixmap.loadFromData(packet):
            return

        parent.ui.label_display.setPixmap(pixmap)
from handler.opcode import Opcode

from PyQt6.QtWidgets import QMessageBox
from PySide6.QtWidgets import QTableWidgetItem

class PacketHandler():
    def handle_packet(socket, parent, packet):
        opcode = packet.read_opcode()

        if opcode == Opcode.SEND_RESIDENT_INFO.value:
            status = packet.read_status()
            
            if status == 0x00:
                QMessageBox.information(None, "성공", "신규 입소자가 등록되었습니다.")
            elif status == 0x01:
                QMessageBox.warning(None, "실패", "이미 등록되어 있는 입소자입니다.")
            elif status == 0xFF:
                QMessageBox.warning(None, "실패", "신규 입소자 등록에 실패했습니다. 다시 시도해주세요.")

        elif opcode == Opcode.RESIDENT_LIST.value:
            status = packet.read_status()

            if status == 0x00:
                parent.ui.tbResidentList.clearContents()
                parent.ui.tbResidentList.setRowCount(0)
                for idx in range(packet.read_int()):
                    parent.ui.tbResidentList.insertRow(idx)
                    name = packet.read_string()
                    birthday = packet.read_string()

                    parent.ui.tbResidentList.setItem(idx, 0, QTableWidgetItem(name))
                    parent.ui.tbResidentList.setItem(idx, 1, QTableWidgetItem(birthday))
            elif status == 0xFF:
                QMessageBox.warning(None, "에러", "입소자 목록 조회에 실패했습니다. 다시 시도해주세요.")
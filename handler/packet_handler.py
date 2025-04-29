from handler.opcode import Opcode
from network.packet import Packet

from PySide6.QtWidgets import QTableWidgetItem, QMessageBox, QFrame

class PacketHandler():
    def handle_packet(socket, parent, packet):
        opcode = packet.read_opcode()

        if opcode == Opcode.SEND_RESIDENT_INFO.value:
            status = packet.read_status()
            
            if status == 0x00:
                QMessageBox.information(None, "성공", "신규 입소자가 등록되었습니다.")
                socket.sendData(Packet.request_resident_list())
                parent.ui.btn_info.click()
            elif status == 0x01:
                QMessageBox.warning(None, "실패", "이미 등록되어 있는 입소자입니다.")
            elif status == 0xFF:
                QMessageBox.warning(None, "실패", "신규 입소자 등록에 실패했습니다. 다시 시도해주세요.")

        elif opcode == Opcode.RESIDENT_LIST.value:
            status = packet.read_status()

            if status == 0x00:
                parent.ui.tbResidentList.clearContents()
                parent.ui.tbResidentList.setRowCount(0)
                count = packet.read_int()
                if count == 0:
                    QMessageBox.information(None, "에러", "조회된 입소자가 없습니다.")
                    return
                
                for idx in range(count):
                    parent.ui.tbResidentList.insertRow(idx)
                    name = packet.read_string()
                    birthday = packet.read_string()
                    sex = "남성" if packet.read_char() == 'M' else "여성"

                    parent.ui.tbResidentList.setItem(idx, 0, QTableWidgetItem(name))
                    parent.ui.tbResidentList.setItem(idx, 1, QTableWidgetItem(sex))
                    parent.ui.tbResidentList.setItem(idx, 2, QTableWidgetItem(birthday))
            elif status == 0xFF:
                QMessageBox.warning(None, "에러", "입소자 목록 조회에 실패했습니다. 다시 시도해주세요.")
        elif opcode == Opcode.REQUEST_RESIDENT_INFO.value:
            status = packet.read_status()

            if status == 0x00:
                name = packet.read_string()
                sex = packet.read_char()
                birthday = packet.read_string()
                face = packet.read_image()
                
                parent.ui.label_8.setPixmap(face)
                parent.ui.label_8.setScaledContents(True)
                parent.ui.label_8.setFrameShape(QFrame.Shape.NoFrame)

                parent.ui.lineEdit_4.setText(name)
                index = 0 if sex == 'M' else 1
                parent.ui.combo_sex_2.setCurrentIndex(index)
            else:
                QMessageBox.warning(None, "에러", "입소자 상세조회에 실패했습니다. 다시 시도해주세요.")
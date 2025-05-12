from handler.opcode import Opcode
from network.packet import Packet
from modules import UIFunctions

from PySide6.QtWidgets import QTableWidgetItem, QMessageBox, QFrame
from PySide6.QtCore import QDate, Qt
from PySide6.QtNetwork import QNetworkInterface, QAbstractSocket

class PacketHandler():
    def handle_packet(socket, parent, packet):
        opcode = packet.read_byte()

        if opcode == Opcode.CLIENT_HELLO.value:
            parent.socket.sendData(Packet.request_robot_list())
        elif opcode == Opcode.SEND_RESIDENT_INFO.value:
            status = packet.read_byte()
            
            if status == 0x00:
                QMessageBox.information(None, "성공", "신규 입소자가 등록되었습니다.")
                socket.sendData(Packet.request_resident_list())
                parent.ui.btn_info.click()
            elif status == 0x01:
                QMessageBox.warning(None, "실패", "이미 등록되어 있는 입소자입니다.")
            elif status == 0xFF:
                QMessageBox.warning(None, "실패", "신규 입소자 등록에 실패했습니다. 다시 시도해주세요.")

        elif opcode == Opcode.RESIDENT_LIST.value:
            status = packet.read_byte()

            if status == 0x00:
                parent.ui.tbResidentList.clearContents()
                parent.ui.tbResidentList.setRowCount(0)
                count = packet.read_short()
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
            status = packet.read_byte()

            if status == 0x00:
                name = packet.read_string()
                birthday = QDate.fromString(packet.read_string(), 'yyyy-MM-dd')
                sex = packet.read_char()
                face = packet.read_image()

                parent.ui.label_8.setPixmap(face)
                parent.ui.label_8.setScaledContents(True)
                parent.ui.label_8.setFrameShape(QFrame.Shape.NoFrame)

                parent.ui.lineEdit_4.setText(name)
                index = 0 if sex == 'M' else 1
                parent.ui.combo_sex_2.setCurrentIndex(index)
                parent.ui.date_birth_2.setDate(birthday)
            else:
                QMessageBox.warning(None, "에러", "입소자 상세조회에 실패했습니다. 다시 시도해주세요.")
        elif opcode == Opcode.REQUEST_DISCHARGE.value:
            status = packet.read_byte()

            if status == 0x00:
                name = packet.read_string()

                QMessageBox.information(None, "성공", name + "님을 퇴소처리하였습니다.")
            else:
                QMessageBox.warning(None, "에러", "퇴소처리에 실패했습니다. 다시 시도해주세요.")
        elif opcode == Opcode.UPDATE_RESIDENT_INFO.value:
            status = packet.read_byte()

            if status == 0x00:
                QMessageBox.information(None, "성공", "정보를 수정했습니다.")
                parent.socket.sendData(Packet.request_resident_list())
            else:
                QMessageBox.warning(None, "에러", "입소자 정보 수정에 실패했습니다. 다시 시도해주세요.")
        elif opcode == Opcode.DELETE_RESIDENT.value:
            status = packet.read_byte()

            if status == 0x00:
                name = parent.ui.lineEdit_4.text()
                QMessageBox.information(None, "성공", name + "님의 정보가 삭제되었습니다.")
                parent.socket.sendData(Packet.request_resident_list())
            else:
                QMessageBox.warning(None, "에러", "입소자 정보 삭제에 실패했습니다. 다시 시도해주세요.")
        elif opcode == Opcode.LOG_CATEGORY.value:
            status = packet.read_byte()

            if status == 0x00:
                parent.ui.comboBox.clear()
                parent.ui.comboBox_2.clear()
                parent.ui.comboBox.addItem("전체")
                parent.ui.comboBox_2.addItem("전체")
                for type in range(packet.read_byte()):
                    parent.ui.comboBox.addItem(packet.read_string())

                for name in range(packet.read_byte()):
                    parent.ui.comboBox_2.addItem(packet.read_string())
            else:
                QMessageBox.warning(None, "에러", "알 수 없는 에러가 발생했습니다. 다시 시도해주세요.")
        elif opcode == Opcode.LOG_SEARCH.value:
            status = packet.read_byte()

            if status == 0x00:
                parent.ui.tableWidget.clearContents()
                parent.ui.tableWidget.setRowCount(0)
                count = packet.read_short()
                if count == 0:
                    QMessageBox.warning(None, "에러", "조회된 로그가 없습니다.")
                    return
                
                for idx in range(count):
                    parent.ui.tableWidget.insertRow(idx)
                    type = packet.read_string()
                    robot_id = packet.read_byte()
                    comment = packet.read_string()
                    date = packet.read_string()

                    parent.ui.tableWidget.setItem(idx, 0, QTableWidgetItem(str(idx)))
                    parent.ui.tableWidget.setItem(idx, 1, QTableWidgetItem(type))
                    parent.ui.tableWidget.setItem(idx, 2, QTableWidgetItem(parent.ui.comboBox_2.itemText(robot_id)))
                    parent.ui.tableWidget.setItem(idx, 3, QTableWidgetItem(comment))
                    parent.ui.tableWidget.setItem(idx, 4, QTableWidgetItem(date))
            else:
                QMessageBox.warning(None, "에러", "알 수 없는 에러가 발생했습니다. 다시 시도해주세요.")
        elif opcode == Opcode.DETECTION.value:
            if parent.is_emergency:
                return
            
            robot_id = packet.read_byte()
            type = packet.read_string()

            retval = QMessageBox.critical(None, "경고", str(robot_id) + "번 로봇이 '" + type + "' 감지!!\n카메라를 연결하시겠습니까?", QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
            if retval == QMessageBox.Yes:
                parent.socket.sendData(Packet.request_video(parent.addr, robot_id))
                parent.is_emergency = True

        elif opcode == Opcode.ROBOT_LIST.value:
            size = packet.read_byte()
            robots = []

            for index in range(size):
                id = packet.read_byte()
                status = packet.read_string()
                battery = packet.read_byte()
                online = "online" if packet.read_bool() else "offline"

                robots.append({'index' : index, 'name' : f"Robot {id}", 'status' : status, 'battery' : f"{battery}%", 'online' : online})

            UIFunctions.build_robot_list(parent, robots)
        elif opcode == Opcode.ROBOT_LOCATION.value:
            size = packet.read_short()

            if parent.ui.stackedWidget.currentWidget().objectName() in ["home", "page"]:
                parent.ui.map.clear_markers()
                for idx in range(size):
                    x = packet.read_short()
                    y = packet.read_short()

                    parent.ui.map.draw_marker(x, y, f"누리{idx}호")
        elif opcode == Opcode.PATROL_LIST.value:
            size = packet.read_byte()

            parent.ui.patrol_table.clearContents()
            parent.ui.patrol_table.setRowCount(0)
            for idx in range(size):
                parent.ui.patrol_table.insertRow(idx)

                time = packet.read_string()

                item = QTableWidgetItem(time)
                item.setTextAlignment(Qt.AlignCenter)
                parent.ui.patrol_table.setItem(idx, 0, item)
        elif opcode == Opcode.PATROL_REGIST.value:
            status = packet.read_byte()

            if status == 0x00:
                QMessageBox.information(None, "성공", "순찰 스케쥴이 등록되었습니다.")
                parent.socket.sendData(Packet.request_patrol_schedule())
            else:
                QMessageBox.warning(None, "에러", "알 수 없는 에러가 발생했습니다. 다시 시도해주세요.")
        elif opcode == Opcode.PATROL_UNREGIST.value:
            status = packet.read_byte()

            if status == 0x00:
                QMessageBox.information(None, "성공", "해당 시간의 순찰이 취소되었습니다.")

                parent.socket.sendData(Packet.request_patrol_schedule())
            else:
                QMessageBox.warning(None, "에러", "알 수 없는 에러가 발생했습니다. 다시 시도해주세요.")
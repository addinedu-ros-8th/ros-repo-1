# ///////////////////////////////////////////////////////////////
#
# BY: WANDERSON M.PIMENTA
# PROJECT MADE WITH: Qt Designer and PySide6
# V: 1.0.0
#
# This project can be used freely for all uses, as long as they maintain the
# respective credits only in the Python scripts, any information in the visual
# interface (GUI) can be modified without any implication.
#
# There are limitations on Qt licenses if you want to use your products
# commercially, I recommend reading them on the official website:
# https://doc.qt.io/qtforpython/licenses.html
#
# ///////////////////////////////////////////////////////////////
import math

# MAIN FILE
# ///////////////////////////////////////////////////////////////
from modules import *
from widgets import *
from network.packet import Packet
from PySide6.QtTest import QTest

# GLOBALS
# ///////////////////////////////////////////////////////////////
GLOBAL_STATE = False
GLOBAL_TITLE_BAR = True

class UIFunctions():
    # MAXIMIZE/RESTORE
    # ///////////////////////////////////////////////////////////////
    def maximize_restore(parent):
        global GLOBAL_STATE
        status = GLOBAL_STATE
        if status == False:
            parent.normal_size = parent.size()
            parent.normal_pos = parent.pos()

            parent.showMaximized()
            GLOBAL_STATE = True
            parent.ui.appMargins.setContentsMargins(0, 0, 0, 0)
            parent.ui.maximizeRestoreAppBtn.setToolTip("Restore")
            parent.ui.maximizeRestoreAppBtn.setIcon(QIcon(u":/icons/images/icons/icon_restore.png"))
            parent.ui.frame_size_grip.hide()
            parent.left_grip.hide()
            parent.right_grip.hide()
            parent.top_grip.hide()
            parent.bottom_grip.hide()
        else:
            GLOBAL_STATE = False
            parent.showNormal()
            if hasattr(parent, "normal_size"):
                # parent.resize(parent.width()+1, parent.height()+1)
                parent.resize(parent.normal_size)
            if hasattr(parent, "normal_pos"):
                parent.move(parent.normal_pos)

            parent.ui.appMargins.setContentsMargins(10, 10, 10, 10)
            parent.ui.maximizeRestoreAppBtn.setToolTip("Maximize")
            parent.ui.maximizeRestoreAppBtn.setIcon(QIcon(u":/icons/images/icons/icon_maximize.png"))
            parent.ui.frame_size_grip.show()
            parent.left_grip.show()
            parent.right_grip.show()
            parent.top_grip.show()
            parent.bottom_grip.show()

    # RETURN STATUS
    # ///////////////////////////////////////////////////////////////
    def returStatus(parent):
        return GLOBAL_STATE

    # SET STATUS
    # ///////////////////////////////////////////////////////////////
    def setStatus(parent, status):
        global GLOBAL_STATE
        GLOBAL_STATE = status

    # TOGGLE MENU
    # ///////////////////////////////////////////////////////////////
    def toggleMenu(parent, enable):
        if enable:
            # GET WIDTH
            width = parent.ui.leftMenuBg.width()
            maxExtend = Settings.MENU_WIDTH
            standard = 60

            # SET MAX WIDTH
            if width == 60:
                widthExtended = maxExtend
            else:
                widthExtended = standard

            # ANIMATION
            parent.animation = QPropertyAnimation(parent.ui.leftMenuBg, b"minimumWidth")
            parent.animation.setDuration(Settings.TIME_ANIMATION)
            parent.animation.setStartValue(width)
            parent.animation.setEndValue(widthExtended)
            parent.animation.setEasingCurve(QEasingCurve.InOutQuart)
            parent.animation.start()

    # TOGGLE RIGHT BOX
    # ///////////////////////////////////////////////////////////////
    def toggleRightBox(parent, enable):
        if enable:
            # GET WIDTH
            width = parent.ui.extraRightBox.width()
            widthLeftBox = parent.ui.extraLeftBox.width()
            maxExtend = Settings.RIGHT_BOX_WIDTH
            color = Settings.BTN_RIGHT_BOX_COLOR
            standard = 0

            # GET BTN STYLE
            style = parent.ui.settingsTopBtn.styleSheet()

            # SET MAX WIDTH
            if width == 0:
                widthExtended = maxExtend
                # SELECT BTN
                parent.ui.settingsTopBtn.setStyleSheet(style + color)
                if widthLeftBox != 0:
                    style = parent.ui.toggleLeftBox.styleSheet()
                    parent.ui.toggleLeftBox.setStyleSheet(style.replace(Settings.BTN_LEFT_BOX_COLOR, ''))
            else:
                widthExtended = standard
                # RESET BTN
                parent.ui.settingsTopBtn.setStyleSheet(style.replace(color, ''))

            UIFunctions.start_box_animation(parent, widthLeftBox, width, "right")

    def start_box_animation(parent, left_box_width, right_box_width, direction):
        right_width = 0
        left_width = 0 

        # Check values
        if left_box_width == 0 and direction == "left":
            left_width = 240
        else:
            left_width = 0
        # Check values
        if right_box_width == 0 and direction == "right":
            right_width = 240
        else:
            right_width = 0       

        # ANIMATION LEFT BOX        
        parent.left_box = QPropertyAnimation(parent.ui.extraLeftBox, b"minimumWidth")
        parent.left_box.setDuration(Settings.TIME_ANIMATION)
        parent.left_box.setStartValue(left_box_width)
        parent.left_box.setEndValue(left_width)
        parent.left_box.setEasingCurve(QEasingCurve.InOutQuart)

        # ANIMATION RIGHT BOX        
        parent.right_box = QPropertyAnimation(parent.ui.extraRightBox, b"minimumWidth")
        parent.right_box.setDuration(Settings.TIME_ANIMATION)
        parent.right_box.setStartValue(right_box_width)
        parent.right_box.setEndValue(right_width)
        parent.right_box.setEasingCurve(QEasingCurve.InOutQuart)

        # GROUP ANIMATION
        parent.group = QParallelAnimationGroup()
        parent.group.addAnimation(parent.left_box)
        parent.group.addAnimation(parent.right_box)
        parent.group.start()

    # SELECT/DESELECT MENU
    # ///////////////////////////////////////////////////////////////
    # SELECT
    def selectMenu(getStyle):
        select = getStyle + Settings.MENU_SELECTED_STYLESHEET
        return select

    # DESELECT
    def deselectMenu(getStyle):
        deselect = getStyle.replace(Settings.MENU_SELECTED_STYLESHEET, "")
        return deselect

    # START SELECTION
    def selectStandardMenu(parent, widget):
        for w in parent.ui.topMenu.findChildren(QPushButton):
            if w.objectName() == widget:
                w.setStyleSheet(UIFunctions.selectMenu(w.styleSheet()))

    # RESET SELECTION
    def resetStyle(parent, widget):
        for w in parent.ui.topMenu.findChildren(QPushButton):
            if w.objectName() != widget:
                w.setStyleSheet(UIFunctions.deselectMenu(w.styleSheet()))

    # IMPORT THEMES FILES QSS/CSS
    # ///////////////////////////////////////////////////////////////
    def theme(parent, file, useCustomTheme):
        if useCustomTheme:
            str = open(file, 'r').read()
            parent.ui.styleSheet.setStyleSheet(str)

    # START - GUI DEFINITIONS
    # ///////////////////////////////////////////////////////////////
    def uiDefinitions(parent):
        def dobleClickMaximizeRestore(event):
            # IF DOUBLE CLICK CHANGE STATUS
            if event.type() == QEvent.MouseButtonDblClick:
                QTimer.singleShot(250, lambda: UIFunctions.maximize_restore(parent))
        parent.ui.titleRightInfo.mouseDoubleClickEvent = dobleClickMaximizeRestore

        if Settings.ENABLE_CUSTOM_TITLE_BAR:
            #STANDARD TITLE BAR
            parent.setWindowFlags(Qt.FramelessWindowHint)
            parent.setAttribute(Qt.WA_TranslucentBackground)

            # MOVE WINDOW / MAXIMIZE / RESTORE
            def moveWindow(event):
                # IF MAXIMIZED CHANGE TO NORMAL
                if UIFunctions.returStatus(parent):
                    UIFunctions.maximize_restore(parent)
                # MOVE WINDOW
                if event.buttons() == Qt.LeftButton:
                    parent.move(parent.pos() + event.globalPos() - parent.dragPos)
                    parent.dragPos = event.globalPos()
                    event.accept()
            parent.ui.titleRightInfo.mouseMoveEvent = moveWindow

            # CUSTOM GRIPS
            parent.left_grip = CustomGrip(parent, Qt.LeftEdge, True)
            parent.right_grip = CustomGrip(parent, Qt.RightEdge, True)
            parent.top_grip = CustomGrip(parent, Qt.TopEdge, True)
            parent.bottom_grip = CustomGrip(parent, Qt.BottomEdge, True)

        else:
            parent.ui.appMargins.setContentsMargins(0, 0, 0, 0)
            parent.ui.minimizeAppBtn.hide()
            parent.ui.maximizeRestoreAppBtn.hide()
            parent.ui.closeAppBtn.hide()
            parent.ui.frame_size_grip.hide()

        # DROP SHADOW
        parent.shadow = QGraphicsDropShadowEffect(parent)
        parent.shadow.setBlurRadius(17)
        parent.shadow.setXOffset(0)
        parent.shadow.setYOffset(0)
        parent.shadow.setColor(QColor(0, 0, 0, 150))
        parent.ui.bgApp.setGraphicsEffect(parent.shadow)

        # RESIZE WINDOW
        parent.sizegrip = QSizeGrip(parent.ui.frame_size_grip)
        parent.sizegrip.setStyleSheet("width: 20px; height: 20px; margin 0px; padding: 0px;")

        # MINIMIZE
        parent.ui.minimizeAppBtn.clicked.connect(lambda: parent.showMinimized())

        # MAXIMIZE/RESTORE
        parent.ui.maximizeRestoreAppBtn.clicked.connect(lambda: UIFunctions.maximize_restore(parent))

        # CLOSE APPLICATION
        parent.ui.closeAppBtn.clicked.connect(lambda: parent.close())

    def resize_grips(parent):
        if Settings.ENABLE_CUSTOM_TITLE_BAR:
            parent.left_grip.setGeometry(0, 10, 10, parent.height())
            parent.right_grip.setGeometry(parent.width() - 10, 10, 10, parent.height())
            parent.top_grip.setGeometry(0, 0, parent.width(), 10)
            parent.bottom_grip.setGeometry(0, parent.height() - 10, parent.width(), 10)

    # ///////////////////////////////////////////////////////////////
    # END - GUI DEFINITIONS

    def setComponentEnabled(parent, enabled):
        parent.ui.toggleButton.setEnabled(enabled)    
        parent.ui.btn_home.setEnabled(enabled)
        parent.ui.btn_residents.setEnabled(enabled)
        parent.ui.btn_logs.setEnabled(enabled)
        parent.ui.btn_settings.setEnabled(enabled)

    def loading(parent):
        parent.moive = QMovie(":/icons/images/icons/loading.gif")
        parent.moive.setCacheMode(QMovie.CacheAll)

        parent.ui.movie_label.setMovie(parent.moive)

        parent.ui.lineEdit_2.setVisible(False)

        parent.moive.start()

        parent.ui.stackedWidget.setCurrentWidget(parent.ui.home)
        UIFunctions.resetStyle(parent, "btn_home")
        parent.ui.btn_home.setStyleSheet(UIFunctions.selectMenu(parent.ui.btn_home.styleSheet()))

    def click_info(parent):
        if parent.ui.btn_info.text() == "신규등록":
            parent.ui.btn_info.setText("이전화면")
            parent.ui.stackedWidget_2.setCurrentWidget(parent.ui.add)
        elif parent.ui.btn_info.text() == "이전화면":
            parent.ui.btn_info.setText("신규등록")
            parent.ui.stackedWidget_2.setCurrentWidget(parent.ui.info)
            parent.ui.label_2.clear()
            parent.ui.label_2.setText("사진을 등록해주세요.")
            parent.ui.lineEdit_3.clear()
            parent.ui.date_birth.setDate(QDate.currentDate())

    def click_face(parent):
        file = QFileDialog.getOpenFileName(parent, "사진 선택", "./", "Image Files (*.png *.jpg *.jpeg *.bmp *.gif)")

        if file[0]:
            with open(file[0], 'rb') as f:                
                parent.pixmap = QPixmap(file[0])

                parent.ui.label_2.setPixmap(parent.pixmap)
                parent.ui.label_2.setScaledContents(True)
                parent.ui.label_2.setFrameShape(QFrame.Shape.NoFrame)

    def click_push(parent):
        if parent.ui.lineEdit_3.text() == "":
            QMessageBox.warning(parent, "실패", "이름을 입력해주세요.")
            parent.ui.lineEdit_3.setFocus()
            return
        face = parent.ui.label_2.pixmap()
        if face.isNull():
            QMessageBox.warning(parent, "실패", "사진을 등록해주세요.")
            return
        
        name = parent.ui.lineEdit_3.text()
        birthday = parent.ui.date_birth.text()
        sex = 'M' if parent.ui.combo_sex.currentText() == "남성" else 'F'
        room_number = 101 if parent.ui.combo_room.currentIndex() == 0 \
            else 102 if parent.ui.combo_room.currentIndex() == 1 else 103
        bed_number = int(parent.ui.bed_number.text())
        
        parent.socket.sendData(Packet.send_resident_info(
            name, birthday, sex, room_number, bed_number, face
        ))

    def click_table(parent):
        parent.ui.info.setEnabled(True)
        selected_indexes = parent.ui.tbResidentList.selectionModel().selectedRows()
        if not selected_indexes:
            return
        
        row = selected_indexes[0].row()
        name = parent.ui.tbResidentList.item(row, 0).text()
        birth = parent.ui.tbResidentList.item(row, 2).text()

        parent.socket.sendData(Packet.request_resident_info(name, birth))

    def click_discharge(parent):
        name = parent.ui.lineEdit_4.text()
        retval = QMessageBox.question(parent, "재확인", name + "님을 퇴소처리 하시겠습니까?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        
        if retval == QMessageBox.Yes:
            birthday = parent.ui.date_birth_2.text()
            
            parent.socket.sendData(Packet.request_discharge(name, birthday))

    def click_search(parent):
        name = parent.ui.lineEdit.text()
        check = parent.ui.check_discharge.isChecked()

        parent.socket.sendData(Packet.request_resident_list(name, check))
    
    def click_modify(parent):
        name = parent.ui.lineEdit_4.text()
        sex = 'M' if parent.ui.combo_sex_2.currentText() == "남성" else 'F'
        birthday = parent.ui.date_birth_2.text()

        parent.socket.sendData(Packet.update_resident_info(
            name, birthday, sex, 0, 0
        ))

    def click_delete(parent):
        name = parent.ui.lineEdit_4.text()
        birthday = parent.ui.date_birth_2.text()

        parent.socket.sendData(Packet.delete_resident_info(name, birthday))

    def build_robot_list(parent, robots: list[dict]):
        layout = parent.ui.robotListLayout

        # 기존 항목 제거
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        for robot in robots:
            widget = ui_main.RobotEntry(robot)
            widget.clicked.connect(lambda name: UIFunctions.handle_robot_click(parent, name))
            layout.addWidget(widget)

        layout.setSpacing(10)
        layout.addItem(QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))

    def clear_field(parent):
        parent.ui.lineEdit_4.clear()
        parent.ui.combo_sex_2.setCurrentIndex(0)
        parent.ui.date_birth_2.setDate(QDate.currentDate())
        parent.ui.combo_room_2.setCurrentIndex(0)
        parent.ui.bed_number_2.setValue(1)
        parent.ui.label_8.clear()
        parent.ui.label_8.setFrameShape(QFrame.Shape.Box)
        parent.ui.info.setEnabled(False)

    def click_refresh(parent):
        parent.socket.sendData(Packet.request_robot_list())

    def click_log_search(parent):
        start = parent.ui.dateEdit.text()
        end = parent.ui.dateEdit_2.text()
        event_type = parent.ui.comboBox.currentText()
        robot = parent.ui.comboBox_2.currentIndex()
        keyword = parent.ui.line_log.text()

        parent.socket.sendData(Packet.request_log_list(start, end, event_type, robot, keyword))

    def handle_robot_click(parent, index):
        # parent.socket.sendData(Packet.request_video('done', 1))
        if parent.is_emergency:
            retval = QMessageBox.information(None, "경고", "비상상황을 종료하시겠습니까?", QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)

            if retval == QMessageBox.Yes:
                parent.socket.sendData(Packet.request_video('done', index+1))
                parent.is_emergency = False
                QTest.qWait(100)
                parent.ui.map.set_map()
        elif not parent.is_emergency and parent.ui.robotListLayout.itemAt(index).widget().status_label.text() == "비상상황":
            retval = QMessageBox.information(None, "경고", "카메라를 연결하시겠습니까?", QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
            if retval == QMessageBox.Yes:
                parent.socket.sendData(Packet.request_video(parent.addr, index+1))
            else:
                parent.socket.sendData(Packet.request_video('done', index+1))
                parent.is_emergency = False
                QTest.qWait(100)
                parent.ui.map.set_map()

        # print(parent.ui.robotListLayout.itemAt(index).widget().status_label.text())

    def click_map(parent, x, y):
        label = parent.ui.label_display
        pixmap = label.pixmap()

        x_real, y_real = UIFunctions.label_px_to_world(x, y, parent.ui.map.map_info)

        image = pixmap.toImage()
        color = image.pixelColor(x_real, y_real)

        print(color.name())

        if color.name() == "#000000":
            QMessageBox.warning(parent, "경고", "로봇이 이동할 수 없는 위치입니다.")

        parent.socket.sendData(Packet.send_goal_pose(x_real, y_real))

        print(f"실제 좌표: ({x_real}, {y_real})")

    def label_px_to_world(px, py, map_info, label_width=640, label_height=480):
        resolution = map_info['resolution']
        origin_x = map_info['origin_x']
        origin_y = map_info['origin_y']
        map_w = map_info['width']
        map_h = map_info['height']

        # 비율계산
        scale_x = map_w / label_width
        scale_y = map_h / label_height

        # 픽셀 -> 그리드 셀 좌표 (정수 셀 위치)
        gx = (map_w - (px * scale_x))
        gy = (py * scale_y)  # Y축 반전

        # 셀 좌표 -> 실제 월드 좌표
        wx = gx * resolution + origin_x
        wy = gy * resolution + origin_y

        return wx, wy

    def click_patrol_register(parent):
        time = parent.ui.patrol_time_edit.time().toString("H:mm")

        for row in range(parent.ui.patrol_table.rowCount()):
            item = parent.ui.patrol_table.item(row, 0).text()
            if time == item[:-3]:
                QMessageBox.warning(None, "에러", "이미 등록된 순찰시간입니다.")
                return

        parent.socket.sendData(Packet.regist_partrol(time))

    def click_patrol_cancel(parent):
        item = parent.ui.patrol_table.selectedItems()

        if len(item) == 0:
            QMessageBox.warning(None, "에러", "취소할 시간을 선택해주세요.")
        else:
            time = item[0].text()
            parent.socket.sendData(Packet.unregist_patrol(time))

    def click_walk_register(parent):
        time = parent.ui.walk_time_edit.time().toString("HH:mm")
        name = parent.ui.walk_name_combo.currentText()

        for row in range(parent.ui.walk_table.rowCount()):
            sel_name = parent.ui.walk_table.item(row, 0).text()
            sel_time = parent.ui.walk_table.item(row, 1).text()
            print(sel_time, sel_name, time, name)
            if time == sel_time and name == sel_name:
                QMessageBox.warning(None, "에러", f"{name}님은 산책은 '{time}'에 등록되어 있습니다.")
                return

        parent.socket.sendData(Packet.regist_walk(name, time))

    def click_walk_cancel(parent):
        item = parent.ui.walk_table.selectedItems()

        if len(item) == 0:
            QMessageBox.warning(None, "에러", "취소할 시간을 선택해주세요.")
        else:
            name = item[0].text()
            time = item[1].text()
            parent.socket.sendData(Packet.unregist_walk(name, time))
        

class MapDisplay:
    def __init__(self, label: QLabel, map_path: str):
        self.label = label
        self.original_map = QPixmap(map_path)  # 원본 맵 유지
        self.marker_path = u":/icons/images/icons/robot.png"
        self.marker_widgets = []  # QLabel 마커들 저장
        self.set_map()
        self.map_info = {}

    def update_map_info(self, resolution, origin_x, origin_y, width, height):
        new_info = {
            'resolution': resolution,
            'origin_x': origin_x,
            'origin_y': origin_y,
            'width': width,
            'height': height
        }

        if self.map_info != new_info:
            self.map_info = new_info

    def set_map(self):
        """원본 맵만 표시"""
        self.label.setPixmap(self.original_map)
        self.clear_markers()

    def draw_marker(self, x, y, q_x, q_y, q_z, q_w, name: str = ""):
        # 마커 QLabel 생성
        if len(self.map_info) == 0:
            return
        
        pixmap = QPixmap(self.marker_path)
        
        px, py = self.world_to_pixel(x, y, self.map_info)
        yaw = self.quaternion_to_yaw(q_x, q_y, q_z, q_w)
        angle_deg = math.degrees(yaw)
        rotated = self.rotate_pixmap(pixmap, angle_deg)

        marker = QLabel(self.label)
        marker.setPixmap(rotated)
        marker.setToolTip(name)
        marker.setFixedSize(24, 24)
        marker.move(px - 24 - rotated.width() // 2, py -24 - rotated.height() // 2)
        marker.setAttribute(Qt.WA_TranslucentBackground)
        marker.show()

        self.marker_widgets.append(marker)

    def clear_markers(self):
        for marker in self.marker_widgets:
            marker.deleteLater()
        self.marker_widgets.clear()

    def world_to_pixel(self, wx, wy, map_info, label_width=640, label_height=480):
        resolution = map_info['resolution']
        origin_x = map_info['origin_x']
        origin_y = map_info['origin_y']
        map_w = map_info['width']
        map_h = map_info['height']

        # 월드 좌표 → 그리드 인덱스
        gx = (wx - origin_x) / resolution
        gy = (wy - origin_y) / resolution

        # 그리드 인덱스 → 픽셀 좌표 (Y축 반전)
        scale_x = label_width / map_w
        scale_y = label_height / map_h

        px = int((map_w - gx) * scale_x) # X축 반전
        py = int((gy) * scale_y)

        return px, py
    
    def rotate_pixmap(self, pixmap: QPixmap, angle_degrees: float) -> QPixmap:
        transform = QTransform().rotate(angle_degrees)
        return pixmap.transformed(transform, mode=Qt.SmoothTransformation)

    
    def quaternion_to_yaw(self, q_x, q_y, q_z, q_w):
        siny_cosp = 2.0 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        return math.atan2(siny_cosp, cosy_cosp)  # 라디안

    def send_destination(self, robot_id, x, y):
        pub = self.create_publiser
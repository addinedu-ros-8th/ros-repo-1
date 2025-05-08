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

# MAIN FILE
# ///////////////////////////////////////////////////////////////
from modules import *
from widgets import *
from network import packet

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
        
        parent.socket.sendData(packet.Packet.send_resident_info(
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

        parent.socket.sendData(packet.Packet.request_resident_info(name, birth))

    def click_discharge(parent):
        name = parent.ui.lineEdit_4.text()
        retval = QMessageBox.question(parent, "재확인", name + "님을 퇴소처리 하시겠습니까?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        
        if retval == QMessageBox.Yes:
            birthday = parent.ui.date_birth_2.text()
            
            parent.socket.sendData(packet.Packet.request_discharge(name, birthday))

    def click_search(parent):
        name = parent.ui.lineEdit.text()
        check = parent.ui.check_discharge.isChecked()

        parent.socket.sendData(packet.Packet.request_resident_list(name, check))
    
    def click_modify(parent):
        name = parent.ui.lineEdit_4.text()
        sex = 'M' if parent.ui.combo_sex_2.currentText() == "남성" else 'F'
        birthday = parent.ui.date_birth_2.text()

        parent.socket.sendData(packet.Packet.update_resident_info(
            name, birthday, sex, 0, 0
        ))

    def click_delete(parent):
        name = parent.ui.lineEdit_4.text()
        birthday = parent.ui.date_birth_2.text()

        parent.socket.sendData(packet.Packet.delete_resident_info(name, birthday))

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
        parent.socket.sendData(packet.Packet.request_robot_list())

    def handle_robot_click(parent, index):
        pass
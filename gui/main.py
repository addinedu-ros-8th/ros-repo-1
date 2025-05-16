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

import sys
import os

# IMPORT / GUI AND MODULES AND WIDGETS
# ///////////////////////////////////////////////////////////////
from modules import *
from widgets import *
os.environ["QT_FONT_DPI"] = "96" # FIX Problem for High DPI and Scale above 100%

# SET AS GLOBAL WIDGETS
# ///////////////////////////////////////////////////////////////
widgets = None

from network.tcp_socket import TCPSocket
from network.udp_socket import UDPSocket
from network.packet import Packet
from handler.packet_handler import PacketHandler
from handler.emergency_handler import EmergencyHandler

class MainWindow(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)

        # SET AS GLOBAL WIDGETS
        # ///////////////////////////////////////////////////////////////
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        global widgets
        widgets = self.ui

        self.socket = TCPSocket()
        self.socket.receive_data.connect(lambda reader: PacketHandler.handle_packet(self.socket, self, reader))
        if not self.socket.connectToServer("192.168.0.43", 9999):
            QMessageBox.warning(self, "에러", "서버에 연결 할 수 없습니다.")
            sys.exit(0)
        else:
            self.socket.sendData(Packet.client_hello())

        self.udp_socket = UDPSocket()
        self.udp_socket.receive_image.connect(lambda reader: EmergencyHandler.handle_packet(self, reader))
        self.is_emergency = False
        self.emergency = False
        # self.addr = None
        # USE CUSTOM TITLE BAR | USE AS "False" FOR MAC OR LINUX
        # ///////////////////////////////////////////////////////////////
        Settings.ENABLE_CUSTOM_TITLE_BAR = True

        # APP NAME
        # ///////////////////////////////////////////////////////////////
        title = "누리봇"
        description = "요양보호사 케어 누리봇"
        # APPLY TEXTS
        self.setWindowTitle(title)
        widgets.titleRightInfo.setText(description)

        # TOGGLE MENU
        # ///////////////////////////////////////////////////////////////
        widgets.toggleButton.clicked.connect(lambda: UIFunctions.toggleMenu(self, True))

        # SET UI DEFINITIONS
        # ///////////////////////////////////////////////////////////////
        UIFunctions.uiDefinitions(self)

        # BUTTONS CLICK
        # ///////////////////////////////////////////////////////////////

        # LEFT MENUS
        widgets.btn_home.clicked.connect(self.buttonClick)
        widgets.btn_residents.clicked.connect(self.buttonClick)
        widgets.btn_logs.clicked.connect(self.buttonClick)
        widgets.btn_settings.clicked.connect(self.buttonClick)

        # SHOW APP
        # ///////////////////////////////////////////////////////////////
        self.show()

        # SET CUSTOM THEME
        # ///////////////////////////////////////////////////////////////
        useCustomTheme = True
        themeFile = "themes/py_dracula_dark.qss"

        # SET THEME AND HACKS
        if useCustomTheme:
            # LOAD AND APPLY STYLE
            UIFunctions.theme(self, themeFile, True)

            # SET HACKS
            AppFunctions.setThemeHack(self)

        # SET HOME PAGE AND SELECT MENU
        # ///////////////////////////////////////////////////////////////
        widgets.stackedWidget.setCurrentWidget(widgets.page)

        UIFunctions.setComponentEnabled(self, False)
        widgets.lineEdit_2.setFocus()

        widgets.lineEdit_2.returnPressed.connect(self.checkPassword)
        # PacketHandler.signal.connect(self.updateGUI)

    def checkPassword(self):
        password = widgets.lineEdit_2.text()
        if password == "1234":
            if len(self.ui.map.map_info) == 0:
                QMessageBox.warning(self, "에러", "아직 로봇의 준비가 완료되지 않았습니다.\n잠시후 다시 시도해주세요.")
                return
            UIFunctions.setComponentEnabled(self, True)
            widgets.lineEdit_2.setText("")
            # widgets.stackedWidget.setCurrentWidget(widgets.home)
            # widgets.btn_dashboard.setStyleSheet(UIFunctions.selectMenu(widgets.btn_dashboard.styleSheet()))

            UIFunctions.loading(self)
        elif password == "":
            widgets.lineEdit_2.setFocus()
            QMessageBox.warning(self, "실패", "비밀번호를 입력해주세요.")
        else:
            widgets.lineEdit_2.setText("")
            widgets.lineEdit_2.setFocus()
            QMessageBox.warning(self, "실패", "비밀번호가 틀렸습니다.")

    # BUTTONS CLICK
    # Post here your functions for clicked buttons
    # ///////////////////////////////////////////////////////////////
    def buttonClick(self):
        # GET BUTTON CLICKED
        btn = self.sender()
        btnName = btn.objectName()

        # SHOW HOME PAGE
        if btnName == "btn_home":
            widgets.stackedWidget.setCurrentWidget(widgets.home)
            UIFunctions.resetStyle(self, btnName)
            btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))

        # SHOW WIDGETS PAGE
        if btnName == "btn_residents":
            widgets.stackedWidget.setCurrentWidget(widgets.residents)
            UIFunctions.resetStyle(self, btnName)
            btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet()))
            self.socket.sendData(Packet.request_resident_list())
            UIFunctions.clear_field(self)

        # SHOW NEW PAGE
        if btnName == "btn_logs":
            widgets.stackedWidget.setCurrentWidget(widgets.logs) # SET PAGE
            UIFunctions.resetStyle(self, btnName) # RESET ANOTHERS BUTTONS SELECTED
            btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet())) # SELECT MENU
            self.socket.sendData(Packet.request_log_category())

        if btnName == "btn_settings":
            widgets.stackedWidget.setCurrentWidget(widgets.settings) # SET PAGE
            UIFunctions.resetStyle(self, btnName) # RESET ANOTHERS BUTTONS SELECTED
            btn.setStyleSheet(UIFunctions.selectMenu(btn.styleSheet())) # SELECT MENU

            self.socket.sendData(Packet.request_patrol_schedule())
            self.socket.sendData(Packet.request_walk_schedule())
            self.socket.sendData(Packet.request_resident_name_list())


    # RESIZE EVENTS
    # ///////////////////////////////////////////////////////////////
    def resizeEvent(self, event):
        # Update Size Grips
        UIFunctions.resize_grips(self)

    # MOUSE CLICK EVENTS
    # ///////////////////////////////////////////////////////////////
    def mousePressEvent(self, event):
        # SET DRAG POS WINDOW
        self.dragPos = event.globalPosition().toPoint()

if __name__ == "__main__":
    try:
        app = QApplication(sys.argv)
        # app.setWindowIcon(QIcon("icon.ico"))
        window = MainWindow()
    except:
        QMessageBox.critical(None, "에러", "에러가 발생했습니다. 관리자에게 문의 바랍니다.")
    finally:
        sys.exit(app.exec())

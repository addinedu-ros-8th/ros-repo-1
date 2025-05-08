# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'maingnxZcz.ui'
##
## Created by: Qt User Interface Compiler version 6.0.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *

from . resources_rc import *

from modules import *

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(940, 560)
        MainWindow.setMinimumSize(QSize(940, 560))
        MainWindow.setMaximumSize(QSize(940, 560))
        self.styleSheet = QWidget(MainWindow)
        self.styleSheet.setObjectName(u"styleSheet")
        font = QFont()
        font.setFamily(u"Segoe UI")
        font.setPointSize(10)
        font.setBold(False)
        font.setItalic(False)
        self.styleSheet.setFont(font)

        self.appMargins = QVBoxLayout(self.styleSheet)
        self.appMargins.setSpacing(0)
        self.appMargins.setObjectName(u"appMargins")
        self.appMargins.setContentsMargins(10, 10, 10, 10)
        self.bgApp = QFrame(self.styleSheet)
        self.bgApp.setObjectName(u"bgApp")
        self.bgApp.setStyleSheet(u"")
        self.bgApp.setFrameShape(QFrame.NoFrame)
        self.bgApp.setFrameShadow(QFrame.Raised)
        self.appLayout = QHBoxLayout(self.bgApp)
        self.appLayout.setSpacing(0)
        self.appLayout.setObjectName(u"appLayout")
        self.appLayout.setContentsMargins(0, 0, 0, 0)
        self.leftMenuBg = QFrame(self.bgApp)
        self.leftMenuBg.setObjectName(u"leftMenuBg")
        self.leftMenuBg.setMinimumSize(QSize(60, 0))
        self.leftMenuBg.setMaximumSize(QSize(60, 16777215))
        self.leftMenuBg.setFrameShape(QFrame.NoFrame)
        self.leftMenuBg.setFrameShadow(QFrame.Raised)
        self.verticalLayout_3 = QVBoxLayout(self.leftMenuBg)
        self.verticalLayout_3.setSpacing(0)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.topLogoInfo = QFrame(self.leftMenuBg)
        self.topLogoInfo.setObjectName(u"topLogoInfo")
        self.topLogoInfo.setMinimumSize(QSize(0, 50))
        self.topLogoInfo.setMaximumSize(QSize(16777215, 50))
        self.topLogoInfo.setFrameShape(QFrame.NoFrame)
        self.topLogoInfo.setFrameShadow(QFrame.Raised)
        self.topLogo = QFrame(self.topLogoInfo)
        self.topLogo.setObjectName(u"topLogo")
        self.topLogo.setGeometry(QRect(10, 5, 42, 42))
        self.topLogo.setMinimumSize(QSize(42, 42))
        self.topLogo.setMaximumSize(QSize(42, 42))
        self.topLogo.setFrameShape(QFrame.NoFrame)
        self.topLogo.setFrameShadow(QFrame.Raised)
        self.titleLeftApp = QLabel(self.topLogoInfo)
        self.titleLeftApp.setObjectName(u"titleLeftApp")
        self.titleLeftApp.setGeometry(QRect(70, 8, 160, 20))
        font1 = QFont()
        font1.setFamily(u"Segoe UI Semibold")
        font1.setPointSize(12)
        font1.setBold(False)
        font1.setItalic(False)
        self.titleLeftApp.setFont(font1)
        self.titleLeftApp.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignTop)
        self.titleLeftDescription = QLabel(self.topLogoInfo)
        self.titleLeftDescription.setObjectName(u"titleLeftDescription")
        self.titleLeftDescription.setGeometry(QRect(70, 27, 160, 16))
        self.titleLeftDescription.setMaximumSize(QSize(16777215, 16))
        font2 = QFont()
        font2.setFamily(u"Segoe UI")
        font2.setPointSize(8)
        font2.setBold(False)
        font2.setItalic(False)
        self.titleLeftDescription.setFont(font2)
        self.titleLeftDescription.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignTop)

        self.verticalLayout_3.addWidget(self.topLogoInfo)

        self.leftMenuFrame = QFrame(self.leftMenuBg)
        self.leftMenuFrame.setObjectName(u"leftMenuFrame")
        self.leftMenuFrame.setFrameShape(QFrame.NoFrame)
        self.leftMenuFrame.setFrameShadow(QFrame.Raised)
        self.verticalMenuLayout = QVBoxLayout(self.leftMenuFrame)
        self.verticalMenuLayout.setSpacing(0)
        self.verticalMenuLayout.setObjectName(u"verticalMenuLayout")
        self.verticalMenuLayout.setContentsMargins(0, 0, 0, 0)
        self.toggleBox = QFrame(self.leftMenuFrame)
        self.toggleBox.setObjectName(u"toggleBox")
        self.toggleBox.setMaximumSize(QSize(16777215, 45))
        self.toggleBox.setFrameShape(QFrame.NoFrame)
        self.toggleBox.setFrameShadow(QFrame.Raised)
        self.verticalLayout_4 = QVBoxLayout(self.toggleBox)
        self.verticalLayout_4.setSpacing(0)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.toggleButton = QPushButton(self.toggleBox)
        self.toggleButton.setObjectName(u"toggleButton")
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.toggleButton.sizePolicy().hasHeightForWidth())
        self.toggleButton.setSizePolicy(sizePolicy)
        self.toggleButton.setMinimumSize(QSize(0, 45))
        self.toggleButton.setFont(font)
        self.toggleButton.setCursor(QCursor(Qt.PointingHandCursor))
        self.toggleButton.setLayoutDirection(Qt.LeftToRight)
        self.toggleButton.setStyleSheet(u"background-image: url(:/icons/images/icons/icon_menu.png);")

        self.verticalLayout_4.addWidget(self.toggleButton)


        self.verticalMenuLayout.addWidget(self.toggleBox)

        self.topMenu = QFrame(self.leftMenuFrame)
        self.topMenu.setObjectName(u"topMenu")
        self.topMenu.setFrameShape(QFrame.NoFrame)
        self.topMenu.setFrameShadow(QFrame.Raised)
        self.verticalLayout_8 = QVBoxLayout(self.topMenu)
        self.verticalLayout_8.setSpacing(0)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.verticalLayout_8.setContentsMargins(0, 0, 0, 0)

        self.btn_home = QPushButton(self.topMenu)
        self.btn_home.setObjectName(u"btn_home")
        sizePolicy.setHeightForWidth(self.btn_home.sizePolicy().hasHeightForWidth())
        self.btn_home.setSizePolicy(sizePolicy)
        self.btn_home.setMinimumSize(QSize(0, 45))
        self.btn_home.setFont(font)
        self.btn_home.setCursor(QCursor(Qt.PointingHandCursor))
        self.btn_home.setLayoutDirection(Qt.LeftToRight)
        self.btn_home.setStyleSheet(u"background-image: url(:/icons/images/icons/cil-home.png);")

        self.verticalLayout_8.addWidget(self.btn_home)

        self.btn_residents = QPushButton(self.topMenu)
        self.btn_residents.setObjectName(u"btn_residents")
        sizePolicy.setHeightForWidth(self.btn_residents.sizePolicy().hasHeightForWidth())
        self.btn_residents.setSizePolicy(sizePolicy)
        self.btn_residents.setMinimumSize(QSize(0, 45))
        self.btn_residents.setFont(font)
        self.btn_residents.setCursor(QCursor(Qt.PointingHandCursor))
        self.btn_residents.setLayoutDirection(Qt.LeftToRight)
        self.btn_residents.setStyleSheet(u"background-image: url(:/icons/images/icons/cil-people.png);")

        self.verticalLayout_8.addWidget(self.btn_residents)

        self.btn_logs = QPushButton(self.topMenu)
        self.btn_logs.setObjectName(u"btn_logs")
        sizePolicy.setHeightForWidth(self.btn_logs.sizePolicy().hasHeightForWidth())
        self.btn_logs.setSizePolicy(sizePolicy)
        self.btn_logs.setMinimumSize(QSize(0, 45))
        self.btn_logs.setFont(font)
        self.btn_logs.setCursor(QCursor(Qt.PointingHandCursor))
        self.btn_logs.setLayoutDirection(Qt.LeftToRight)
        self.btn_logs.setStyleSheet(u"background-image: url(:/icons/images/icons/cil-notes.png);")

        self.verticalLayout_8.addWidget(self.btn_logs)

        self.btn_settings = QPushButton(self.topMenu)
        self.btn_settings.setObjectName(u"btn_settings")
        sizePolicy.setHeightForWidth(self.btn_settings.sizePolicy().hasHeightForWidth())
        self.btn_settings.setSizePolicy(sizePolicy)
        self.btn_settings.setMinimumSize(QSize(0, 45))
        self.btn_settings.setFont(font)
        self.btn_settings.setCursor(QCursor(Qt.PointingHandCursor))
        self.btn_settings.setLayoutDirection(Qt.LeftToRight)
        self.btn_settings.setStyleSheet(u"background-image: url(:/icons/images/icons/cil-settings.png);")

        self.verticalLayout_8.addWidget(self.btn_settings)

        self.verticalMenuLayout.addWidget(self.topMenu, 0, Qt.AlignTop)

        self.bottomMenu = QFrame(self.leftMenuFrame)
        self.bottomMenu.setObjectName(u"bottomMenu")
        self.bottomMenu.setFrameShape(QFrame.NoFrame)
        self.bottomMenu.setFrameShadow(QFrame.Raised)
        self.verticalLayout_9 = QVBoxLayout(self.bottomMenu)
        self.verticalLayout_9.setSpacing(0)
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.verticalLayout_9.setContentsMargins(0, 0, 0, 0)

        self.verticalMenuLayout.addWidget(self.bottomMenu, 0, Qt.AlignBottom)


        self.verticalLayout_3.addWidget(self.leftMenuFrame)


        self.appLayout.addWidget(self.leftMenuBg)

        self.extraLeftBox = QFrame(self.bgApp)
        self.extraLeftBox.setObjectName(u"extraLeftBox")
        self.extraLeftBox.setMinimumSize(QSize(0, 0))
        self.extraLeftBox.setMaximumSize(QSize(0, 16777215))
        self.extraLeftBox.setFrameShape(QFrame.NoFrame)
        self.extraLeftBox.setFrameShadow(QFrame.Raised)
        self.extraColumLayout = QVBoxLayout(self.extraLeftBox)
        self.extraColumLayout.setSpacing(0)
        self.extraColumLayout.setObjectName(u"extraColumLayout")
        self.extraColumLayout.setContentsMargins(0, 0, 0, 0)
        self.extraTopBg = QFrame(self.extraLeftBox)
        self.extraTopBg.setObjectName(u"extraTopBg")
        self.extraTopBg.setMinimumSize(QSize(0, 50))
        self.extraTopBg.setMaximumSize(QSize(16777215, 50))
        self.extraTopBg.setFrameShape(QFrame.NoFrame)
        self.extraTopBg.setFrameShadow(QFrame.Raised)
        self.verticalLayout_5 = QVBoxLayout(self.extraTopBg)
        self.verticalLayout_5.setSpacing(0)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.extraTopLayout = QGridLayout()
        self.extraTopLayout.setObjectName(u"extraTopLayout")
        self.extraTopLayout.setHorizontalSpacing(10)
        self.extraTopLayout.setVerticalSpacing(0)
        self.extraTopLayout.setContentsMargins(10, -1, 10, -1)

        self.verticalLayout_5.addLayout(self.extraTopLayout)


        self.extraColumLayout.addWidget(self.extraTopBg)

        self.extraContent = QFrame(self.extraLeftBox)
        self.extraContent.setObjectName(u"extraContent")
        self.extraContent.setFrameShape(QFrame.NoFrame)
        self.extraContent.setFrameShadow(QFrame.Raised)
        self.verticalLayout_12 = QVBoxLayout(self.extraContent)
        self.verticalLayout_12.setSpacing(0)
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.verticalLayout_12.setContentsMargins(0, 0, 0, 0)
        self.extraTopMenu = QFrame(self.extraContent)
        self.extraTopMenu.setObjectName(u"extraTopMenu")
        self.extraTopMenu.setFrameShape(QFrame.NoFrame)
        self.extraTopMenu.setFrameShadow(QFrame.Raised)

        self.verticalLayout_12.addWidget(self.extraTopMenu, 0, Qt.AlignTop)

        self.extraCenter = QFrame(self.extraContent)
        self.extraCenter.setObjectName(u"extraCenter")
        self.extraCenter.setFrameShape(QFrame.NoFrame)
        self.extraCenter.setFrameShadow(QFrame.Raised)


        self.verticalLayout_12.addWidget(self.extraCenter)

        self.extraBottom = QFrame(self.extraContent)
        self.extraBottom.setObjectName(u"extraBottom")
        self.extraBottom.setFrameShape(QFrame.NoFrame)
        self.extraBottom.setFrameShadow(QFrame.Raised)

        self.verticalLayout_12.addWidget(self.extraBottom)


        self.extraColumLayout.addWidget(self.extraContent)


        self.appLayout.addWidget(self.extraLeftBox)

        self.contentBox = QFrame(self.bgApp)
        self.contentBox.setObjectName(u"contentBox")
        self.contentBox.setFrameShape(QFrame.NoFrame)
        self.contentBox.setFrameShadow(QFrame.Raised)
        self.verticalLayout_2 = QVBoxLayout(self.contentBox)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.contentTopBg = QFrame(self.contentBox)
        self.contentTopBg.setObjectName(u"contentTopBg")
        self.contentTopBg.setMinimumSize(QSize(0, 50))
        self.contentTopBg.setMaximumSize(QSize(16777215, 50))
        self.contentTopBg.setFrameShape(QFrame.NoFrame)
        self.contentTopBg.setFrameShadow(QFrame.Raised)
        self.horizontalLayout = QHBoxLayout(self.contentTopBg)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 10, 0)
        self.leftBox = QFrame(self.contentTopBg)
        self.leftBox.setObjectName(u"leftBox")
        sizePolicy1 = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.leftBox.sizePolicy().hasHeightForWidth())
        self.leftBox.setSizePolicy(sizePolicy1)
        self.leftBox.setFrameShape(QFrame.NoFrame)
        self.leftBox.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_3 = QHBoxLayout(self.leftBox)
        self.horizontalLayout_3.setSpacing(0)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.titleRightInfo = QLabel(self.leftBox)
        self.titleRightInfo.setObjectName(u"titleRightInfo")
        sizePolicy2 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.titleRightInfo.sizePolicy().hasHeightForWidth())
        self.titleRightInfo.setSizePolicy(sizePolicy2)
        self.titleRightInfo.setMaximumSize(QSize(16777215, 45))
        self.titleRightInfo.setFont(font)
        self.titleRightInfo.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)

        self.horizontalLayout_3.addWidget(self.titleRightInfo)


        self.horizontalLayout.addWidget(self.leftBox)

        self.rightButtons = QFrame(self.contentTopBg)
        self.rightButtons.setObjectName(u"rightButtons")
        self.rightButtons.setMinimumSize(QSize(0, 28))
        self.rightButtons.setFrameShape(QFrame.NoFrame)
        self.rightButtons.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_2 = QHBoxLayout(self.rightButtons)
        self.horizontalLayout_2.setSpacing(5)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)

        self.minimizeAppBtn = QPushButton(self.rightButtons)
        self.minimizeAppBtn.setObjectName(u"minimizeAppBtn")
        self.minimizeAppBtn.setMinimumSize(QSize(28, 28))
        self.minimizeAppBtn.setMaximumSize(QSize(28, 28))
        self.minimizeAppBtn.setCursor(QCursor(Qt.PointingHandCursor))
        icon2 = QIcon()
        icon2.addFile(u":/icons/images/icons/icon_minimize.png", QSize(), QIcon.Normal, QIcon.Off)
        self.minimizeAppBtn.setIcon(icon2)
        self.minimizeAppBtn.setIconSize(QSize(20, 20))

        self.horizontalLayout_2.addWidget(self.minimizeAppBtn)

        self.maximizeRestoreAppBtn = QPushButton(self.rightButtons)
        self.maximizeRestoreAppBtn.setObjectName(u"maximizeRestoreAppBtn")
        self.maximizeRestoreAppBtn.setMinimumSize(QSize(28, 28))
        self.maximizeRestoreAppBtn.setMaximumSize(QSize(28, 28))
        font3 = QFont()
        font3.setFamily(u"Segoe UI")
        font3.setPointSize(10)
        font3.setBold(False)
        font3.setItalic(False)
        font3.setStyleStrategy(QFont.PreferDefault)
        self.maximizeRestoreAppBtn.setFont(font3)
        self.maximizeRestoreAppBtn.setCursor(QCursor(Qt.PointingHandCursor))
        icon3 = QIcon()
        icon3.addFile(u":/icons/images/icons/icon_maximize.png", QSize(), QIcon.Normal, QIcon.Off)
        self.maximizeRestoreAppBtn.setIcon(icon3)
        self.maximizeRestoreAppBtn.setIconSize(QSize(20, 20))

        self.horizontalLayout_2.addWidget(self.maximizeRestoreAppBtn)

        self.closeAppBtn = QPushButton(self.rightButtons)
        self.closeAppBtn.setObjectName(u"closeAppBtn")
        self.closeAppBtn.setMinimumSize(QSize(28, 28))
        self.closeAppBtn.setMaximumSize(QSize(28, 28))
        self.closeAppBtn.setCursor(QCursor(Qt.PointingHandCursor))
        icon = QIcon()
        icon.addFile(u":/icons/images/icons/icon_close.png", QSize(), QIcon.Normal, QIcon.Off)
        self.closeAppBtn.setIcon(icon)
        self.closeAppBtn.setIconSize(QSize(20, 20))

        self.horizontalLayout_2.addWidget(self.closeAppBtn)


        self.horizontalLayout.addWidget(self.rightButtons, 0, Qt.AlignRight)


        self.verticalLayout_2.addWidget(self.contentTopBg)

        self.contentBottom = QFrame(self.contentBox)
        self.contentBottom.setObjectName(u"contentBottom")
        self.contentBottom.setFrameShape(QFrame.NoFrame)
        self.contentBottom.setFrameShadow(QFrame.Raised)
        self.verticalLayout_6 = QVBoxLayout(self.contentBottom)
        self.verticalLayout_6.setSpacing(0)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.content = QFrame(self.contentBottom)
        self.content.setObjectName(u"content")
        self.content.setFrameShape(QFrame.NoFrame)
        self.content.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_4 = QHBoxLayout(self.content)
        self.horizontalLayout_4.setSpacing(0)
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.pagesContainer = QFrame(self.content)
        self.pagesContainer.setObjectName(u"pagesContainer")
        self.pagesContainer.setStyleSheet(u"")
        self.pagesContainer.setFrameShape(QFrame.NoFrame)
        self.pagesContainer.setFrameShadow(QFrame.Raised)
        self.verticalLayout_15 = QVBoxLayout(self.pagesContainer)
        self.verticalLayout_15.setSpacing(0)
        self.verticalLayout_15.setObjectName(u"verticalLayout_15")
        self.verticalLayout_15.setContentsMargins(10, 10, 10, 10)
        self.stackedWidget = QStackedWidget(self.pagesContainer)
        self.stackedWidget.setObjectName(u"stackedWidget")
        self.stackedWidget.setStyleSheet(u"background: transparent;")
        self.page = QWidget()
        self.page.setObjectName("page")
        self.lineEdit_2 = QLineEdit(self.page)
        self.lineEdit_2.setGeometry(QtCore.QRect(325, 190, 190, 30))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.lineEdit_2.setStyleSheet(u"background-color: rgb(33, 37, 43);")
        self.movie_label = QLabel(self.page)
        self.movie_label.setGeometry(335, 130, 150, 150)
        self.movie_label.setObjectName("movie_label")
        self.stackedWidget.addWidget(self.page)

        self.home = QWidget()
        self.home.setObjectName("home")

        self.homeLayout = QHBoxLayout(self.home)
        self.homeLayout.setContentsMargins(10, 10, 10, 10)
        self.homeLayout.setSpacing(10)

        # 왼쪽 맵/영상 QLabel
        self.label_display = QLabel()
        self.label_display.setObjectName("label_display")
        self.label_display.setStyleSheet("background-color: #222; border: 1px solid #444;")
        self.label_display.setAlignment(Qt.AlignCenter)
        self.label_display.setText("로봇 맵/카메라 영역")
        self.label_display.setMinimumSize(600, 400)
        self.homeLayout.addWidget(self.label_display, 3)  # 왼쪽 영역 (넓게)

        # 오른쪽 전체 컨테이너
        self.robotRightContainer = QWidget()
        self.robotRightLayout = QVBoxLayout(self.robotRightContainer)
        self.robotRightLayout.setContentsMargins(0, 0, 0, 0)
        self.robotRightLayout.setSpacing(10)

        # 상단 새로고침 버튼
        self.refreshButton = QPushButton("새로고침")
        self.refreshButton.setObjectName("refreshButton")
        self.refreshButton.setFixedHeight(30)
        self.robotRightLayout.addWidget(self.refreshButton)

        # 하단: 스크롤 영역 (로봇 리스트)
        self.robotScrollArea = QScrollArea()
        self.robotScrollArea.setObjectName("robotScrollArea")
        self.robotScrollArea.setWidgetResizable(True)
        self.robotScrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.robotScrollArea.setMinimumWidth(200)

        self.robotListContainer = QWidget()
        self.robotListLayout = QVBoxLayout(self.robotListContainer)

        robots = [
        {"index": 0, "name": "Robot 1", "status": "online", "battery": "75%"},
        {"index": 1, "name": "Robot 2", "status": "offline", "battery": "38%"},
        ]

        for robot in robots:
                entry = RobotEntry(robot)
                self.robotListLayout.addWidget(entry)

        self.robotListLayout.addStretch()
        self.robotScrollArea.setWidget(self.robotListContainer)

        # 스크롤 영역을 전체 레이아웃에 추가
        self.robotRightLayout.addWidget(self.robotScrollArea)

        # home 레이아웃에 오른쪽 추가
        self.homeLayout.addWidget(self.robotRightContainer, 1)

        # 페이지 등록
        self.stackedWidget.addWidget(self.home)


        self.residents = QWidget()
        self.residents.setObjectName("residents")
        self.lineEdit = QLineEdit(parent=self.residents)
        self.lineEdit.setGeometry(QtCore.QRect(280, 120, 161, 27))
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit.setClearButtonEnabled(True)
        self.btn_searchinfo = QPushButton(parent=self.residents)
        self.btn_searchinfo.setGeometry(QtCore.QRect(470, 120, 88, 27))
        self.btn_searchinfo.setObjectName("btn_searchinfo")
        self.label_3 = QLabel(parent=self.residents)
        self.label_3.setGeometry(QtCore.QRect(210, 120, 51, 25))
        self.label_3.setObjectName("label_3")
        self.btn_info = QPushButton(parent=self.residents)
        self.btn_info.setGeometry(QtCore.QRect(730, 10, 88, 27))
        self.btn_info.setObjectName("btn_info")
        self.check_discharge = QCheckBox(parent=self.residents)
        self.check_discharge.setGeometry(QtCore.QRect(30, 120, 92, 25))
        self.check_discharge.setObjectName("check_discharge")
        self.resident_info = QGroupBox(parent=self.residents)
        self.resident_info.setGeometry(QtCore.QRect(570, 30, 251, 401))
        self.resident_info.setObjectName("resident_info")
        self.stackedWidget_2 = QStackedWidget(parent=self.resident_info)
        self.stackedWidget_2.setGeometry(QtCore.QRect(0, 30, 251, 371))
        self.stackedWidget_2.setObjectName("stackedWidget_2")

        self.info = QWidget()
        self.info.setObjectName("info")
        self.info.setEnabled(False)
        self.label_8 = QLabel(parent=self.info)
        self.label_8.setGeometry(QtCore.QRect(10, 0, 231, 121))
        self.label_8.setFrameShape(QFrame.Shape.Box)
        self.label_8.setObjectName("label_8")
        self.btn_face_2 = QPushButton(parent=self.info)
        self.btn_face_2.setGeometry(QtCore.QRect(30, 130, 88, 27))
        self.btn_face_2.setObjectName("btn_face_2")
        self.btn_discharge = QPushButton(parent=self.info)
        self.btn_discharge.setGeometry(QtCore.QRect(130, 130, 88, 27))
        self.btn_discharge.setObjectName("btn_discharge")
        self.scrollArea_2 = QScrollArea(parent=self.info)
        self.scrollArea_2.setGeometry(QtCore.QRect(10, 170, 241, 201))
        self.scrollArea_2.setWidgetResizable(True)
        self.scrollArea_2.setObjectName("scrollArea_2")
        self.scrollArea_2.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.scrollAreaWidgetContents_2 = QWidget()
        self.scrollAreaWidgetContents_2.setGeometry(QtCore.QRect(0, 0, 239, 250))
        self.scrollAreaWidgetContents_2.setObjectName("scrollAreaWidgetContents_2")
        self.scrollLayout_2 = QVBoxLayout(self.scrollAreaWidgetContents_2)
        # 이름
        name_layout_2 = QHBoxLayout()
        name_label_2 = QLabel("이름 :", parent=self.scrollAreaWidgetContents_2)
        self.lineEdit_4 = QLineEdit(parent=self.scrollAreaWidgetContents_2)
        name_layout_2.addWidget(name_label_2)
        name_layout_2.addWidget(self.lineEdit_4)
        self.scrollLayout_2.addLayout(name_layout_2)

        # 성별
        sex_layout_2 = QHBoxLayout()
        sex_label_2 = QLabel("성별 :", parent=self.scrollAreaWidgetContents_2)
        self.combo_sex_2 = QComboBox(parent=self.scrollAreaWidgetContents_2)
        self.combo_sex_2.addItems(["남성", "여성"])
        sex_layout_2.addWidget(sex_label_2)
        sex_layout_2.addWidget(self.combo_sex_2)
        self.scrollLayout_2.addLayout(sex_layout_2)

        # 생년월일
        birth_layout_2 = QHBoxLayout()
        birth_label_2 = QLabel("생년월일 :", parent=self.scrollAreaWidgetContents_2)
        self.date_birth_2 = QDateTimeEdit(parent=self.scrollAreaWidgetContents_2)
        self.date_birth_2.setCalendarPopup(True)
        self.date_birth_2.setDate(QDate.currentDate())
        birth_layout_2.addWidget(birth_label_2)
        birth_layout_2.addWidget(self.date_birth_2)
        self.scrollLayout_2.addLayout(birth_layout_2)

        # 호실
        room_layout_2 = QHBoxLayout()
        room_label_2 = QLabel("호실 :", parent=self.scrollAreaWidgetContents_2)
        self.combo_room_2 = QComboBox(parent=self.scrollAreaWidgetContents_2)
        self.combo_room_2.addItems(["1호실", "2호실", "3호실"])
        room_layout_2.addWidget(room_label_2)
        room_layout_2.addWidget(self.combo_room_2)
        self.scrollLayout_2.addLayout(room_layout_2)

        # 침대번호
        bed_layout_2 = QHBoxLayout()
        bed_label_2 = QLabel("침대번호 :", parent=self.scrollAreaWidgetContents_2)
        self.bed_number_2 = QSpinBox(parent=self.scrollAreaWidgetContents_2)
        self.bed_number_2.setMinimum(1)
        self.bed_number_2.setMaximum(5)
        bed_layout_2.addWidget(bed_label_2)
        bed_layout_2.addWidget(self.bed_number_2)
        self.scrollLayout_2.addLayout(bed_layout_2)

        # 수정 버튼
        btn_layout = QHBoxLayout()
        self.btn_push_2 = QPushButton("수정", parent=self.scrollAreaWidgetContents_2)
        btn_layout.addWidget(self.btn_push_2)

        # 삭제 버튼
        self.btn_delete = QPushButton("삭제", parent=self.scrollAreaWidgetContents_2)
        btn_layout.addWidget(self.btn_delete)
        self.scrollLayout_2.addLayout(btn_layout)

        self.scrollArea_2.setWidget(self.scrollAreaWidgetContents_2)
        self.stackedWidget_2.addWidget(self.info)

        self.add = QWidget()
        self.add.setObjectName("add")
        self.label_2 = QLabel(parent=self.add)
        self.label_2.setGeometry(QtCore.QRect(10, 0, 231, 121))
        self.label_2.setFrameShape(QFrame.Shape.Box)
        self.label_2.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.btn_face = QPushButton(parent=self.add)
        self.btn_face.setGeometry(QtCore.QRect(80, 130, 88, 27))
        self.btn_face.setObjectName("btn_face")
        self.scrollArea = QScrollArea(parent=self.add)
        self.scrollArea.setGeometry(QtCore.QRect(10, 170, 241, 201))
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.scrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.scrollAreaWidgetContents = QWidget()
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 239, 250))
        self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.scrollLayout = QVBoxLayout(self.scrollAreaWidgetContents)
        # 이름
        name_layout = QHBoxLayout()
        name_label = QLabel("이름 :", parent=self.scrollAreaWidgetContents)
        self.lineEdit_3 = QLineEdit(parent=self.scrollAreaWidgetContents)
        name_layout.addWidget(name_label)
        name_layout.addWidget(self.lineEdit_3)
        self.scrollLayout.addLayout(name_layout)

        # 성별
        sex_layout = QHBoxLayout()
        sex_label = QLabel("성별 :", parent=self.scrollAreaWidgetContents)
        self.combo_sex = QComboBox(parent=self.scrollAreaWidgetContents)
        self.combo_sex.addItems(["남성", "여성"])
        sex_layout.addWidget(sex_label)
        sex_layout.addWidget(self.combo_sex)
        self.scrollLayout.addLayout(sex_layout)

        # 생년월일
        birth_layout = QHBoxLayout()
        birth_label = QLabel("생년월일 :", parent=self.scrollAreaWidgetContents)
        self.date_birth = QDateTimeEdit(parent=self.scrollAreaWidgetContents)
        self.date_birth.setCalendarPopup(True)
        self.date_birth.setDate(QDate.currentDate())
        birth_layout.addWidget(birth_label)
        birth_layout.addWidget(self.date_birth)
        self.scrollLayout.addLayout(birth_layout)

        # 호실
        room_layout = QHBoxLayout()
        room_label = QLabel("호실 :", parent=self.scrollAreaWidgetContents)
        self.combo_room = QComboBox(parent=self.scrollAreaWidgetContents)
        self.combo_room.addItems(["1호실", "2호실", "3호실"])
        room_layout.addWidget(room_label)
        room_layout.addWidget(self.combo_room)
        self.scrollLayout.addLayout(room_layout)

        # 침대번호
        bed_layout = QHBoxLayout()
        bed_label = QLabel("침대번호 :", parent=self.scrollAreaWidgetContents)
        self.bed_number = QSpinBox(parent=self.scrollAreaWidgetContents)
        self.bed_number.setMinimum(1)
        self.bed_number.setMaximum(5)
        bed_layout.addWidget(bed_label)
        bed_layout.addWidget(self.bed_number)
        self.scrollLayout.addLayout(bed_layout)

        # 등록 버튼
        self.btn_push = QPushButton("등록", parent=self.scrollAreaWidgetContents)
        self.scrollLayout.addWidget(self.btn_push)

        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.stackedWidget_2.addWidget(self.add)

        self.tbResidentList = QTableWidget(parent=self.residents)
        self.tbResidentList.setGeometry(QtCore.QRect(20, 160, 531, 271))
        self.tbResidentList.setObjectName("tbResidentList")
        self.tbResidentList.setColumnCount(4)
        self.tbResidentList.setRowCount(0)
        self.tbResidentList.verticalHeader().setVisible(False)
        self.tbResidentList.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.tbResidentList.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.tbResidentList.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        item = QTableWidgetItem()
        self.tbResidentList.setHorizontalHeaderItem(0, item)
        item = QTableWidgetItem()
        self.tbResidentList.setHorizontalHeaderItem(1, item)
        item = QTableWidgetItem()
        self.tbResidentList.setHorizontalHeaderItem(2, item)
        item = QTableWidgetItem()
        self.tbResidentList.setHorizontalHeaderItem(3, item)
        self.stackedWidget.addWidget(self.residents)

        # 로그 페이지 레이아웃 생성
        self.logs = QWidget()
        self.logs.setObjectName("logs")
        logs_layout = QVBoxLayout(self.logs)

        # 상단 필터 바 생성
        filter_layout = QHBoxLayout()
        self.dateEdit = QDateEdit(calendarPopup=True)
        self.dateEdit.setDate(QDate.currentDate())
        self.dateEdit_2 = QDateEdit(calendarPopup=True)
        self.dateEdit_2.setDate(QDate.currentDate())

        self.comboBox_2 = QComboBox()
        self.comboBox = QComboBox()
        self.line_log = QLineEdit()
        self.btn_log_search = QPushButton("검색")

        filter_layout.addWidget(QLabel("검색 기간"))
        filter_layout.addWidget(self.dateEdit)
        filter_layout.addWidget(QLabel("~"))
        filter_layout.addWidget(self.dateEdit_2)
        filter_layout.addWidget(QLabel("이벤트 타입"))
        filter_layout.addWidget(self.comboBox)
        filter_layout.addWidget(QLabel("로봇"))
        filter_layout.addWidget(self.comboBox_2)
        filter_layout.addWidget(QLabel("검색어"))
        filter_layout.addWidget(self.line_log)
        filter_layout.addWidget(self.btn_log_search)

        # 로그 테이블 생성
        self.tableWidget = QTableWidget(0, 5)
        self.tableWidget.setHorizontalHeaderLabels(["No.", "이벤트 타입", "로봇ID", "로그 내용", "로그 시간"])
        self.tableWidget.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tableWidget.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tableWidget.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tableWidget.horizontalHeader().setStretchLastSection(True)

        # 최종 레이아웃 조립
        logs_layout.addLayout(filter_layout)
        logs_layout.addWidget(self.tableWidget)

        # stackedWidget에 페이지 추가
        self.stackedWidget.addWidget(self.logs)

        # Settings Page 구성
        self.settings = QWidget()
        self.settings.setObjectName("settings")

        # 전체 레이아웃
        self.settingsLayout = QVBoxLayout(self.settings)
        self.settingsLayout.setContentsMargins(0, 0, 0, 0)
        self.settingsLayout.setSpacing(0)

        self.scrollArea = QScrollArea(self.settings)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.settingsLayout.addWidget(self.scrollArea)

        # ScrollArea 내부 컨테이너
        self.scroll_container = QWidget()
        self.scroll_layout = QVBoxLayout(self.scroll_container)
        self.scroll_layout.setContentsMargins(20, 20, 20, 20)
        self.scroll_layout.setSpacing(20)


        # === 순찰 스케줄 영역 ===
        self.group_patrol = QGroupBox("순찰 스케줄")
        self.group_patrol_layout = QVBoxLayout(self.group_patrol)

        # 순찰 등록 부분
        self.patrol_form_layout = QHBoxLayout()
        self.patrol_time_label = QLabel("순찰 시간")
        self.patrol_time_edit = QTimeEdit()
        self.patrol_time_edit.setDisplayFormat("hh:mm AP")
        self.patrol_register_button = QPushButton("등록")
        self.patrol_cancel_button = QPushButton("취소")
        self.patrol_form_layout.addWidget(self.patrol_time_label)
        self.patrol_form_layout.addWidget(self.patrol_time_edit)
        self.patrol_form_layout.addWidget(self.patrol_register_button)
        self.patrol_form_layout.addWidget(self.patrol_cancel_button)
        self.group_patrol_layout.addLayout(self.patrol_form_layout)

        # 순찰 스케줄 테이블
        self.patrol_table = QTableWidget(0, 1)
        self.patrol_table.setHorizontalHeaderLabels(["순찰 스케줄"])
        self.patrol_table.horizontalHeader().setStretchLastSection(True)
        self.group_patrol_layout.addWidget(self.patrol_table)

        # === 산책 스케줄 영역 ===
        self.group_walk = QGroupBox("산책 스케줄", self.settings)
        self.group_walk_layout = QVBoxLayout(self.group_walk)

        # 산책 등록 부분
        self.walk_form_layout = QGridLayout()
        self.walk_time_label = QLabel("산책 시간")
        self.walk_time_edit = QTimeEdit()
        self.walk_time_edit.setDisplayFormat("hh:mm AP")
        self.walk_name_label = QLabel("대상자명")
        self.walk_name_edit = QLineEdit()
        self.walk_name_edit.setFixedWidth(350)
        self.walk_register_button = QPushButton("등록")
        self.walk_cancel_button = QPushButton("취소")

        self.walk_form_layout.addWidget(self.walk_time_label, 0, 0)
        self.walk_form_layout.addWidget(self.walk_time_edit, 0, 1)
        self.walk_form_layout.addWidget(self.walk_name_label, 1, 0)
        self.walk_form_layout.addWidget(self.walk_name_edit, 1, 1)
        self.walk_form_layout.addWidget(self.walk_register_button, 1, 2)
        self.walk_form_layout.addWidget(self.walk_cancel_button, 1, 3)
        self.group_walk_layout.addLayout(self.walk_form_layout)

        # 산책 스케줄 테이블
        self.walk_table = QTableWidget(0, 3)
        self.walk_table.setHorizontalHeaderLabels(["이름", "산책시간", "코스"])
        self.walk_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.group_walk_layout.addWidget(self.walk_table)

        # GroupBox들을 Scroll 내부에 추가
        self.scroll_layout.addWidget(self.group_patrol)
        self.scroll_layout.addWidget(self.group_walk)

        # Spacer for bottom push
        spacer = QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding)
        self.scroll_layout.addItem(spacer)

        # 스크롤영역에 최종 설정
        self.scrollArea.setWidget(self.scroll_container)

        # settings 페이지를 스택에 추가
        self.stackedWidget.addWidget(self.settings)


        self.verticalLayout_15.addWidget(self.stackedWidget)


        self.horizontalLayout_4.addWidget(self.pagesContainer)

        self.extraRightBox = QFrame(self.content)
        self.extraRightBox.setObjectName(u"extraRightBox")
        self.extraRightBox.setMinimumSize(QSize(0, 0))
        self.extraRightBox.setMaximumSize(QSize(0, 16777215))
        self.extraRightBox.setFrameShape(QFrame.NoFrame)
        self.extraRightBox.setFrameShadow(QFrame.Raised)
        self.verticalLayout_7 = QVBoxLayout(self.extraRightBox)
        self.verticalLayout_7.setSpacing(0)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.verticalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.themeSettingsTopDetail = QFrame(self.extraRightBox)
        self.themeSettingsTopDetail.setObjectName(u"themeSettingsTopDetail")
        self.themeSettingsTopDetail.setMaximumSize(QSize(16777215, 3))
        self.themeSettingsTopDetail.setFrameShape(QFrame.NoFrame)
        self.themeSettingsTopDetail.setFrameShadow(QFrame.Raised)

        self.verticalLayout_7.addWidget(self.themeSettingsTopDetail)

        self.contentSettings = QFrame(self.extraRightBox)
        self.contentSettings.setObjectName(u"contentSettings")
        self.contentSettings.setFrameShape(QFrame.NoFrame)
        self.contentSettings.setFrameShadow(QFrame.Raised)
        self.verticalLayout_13 = QVBoxLayout(self.contentSettings)
        self.verticalLayout_13.setSpacing(0)
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.verticalLayout_13.setContentsMargins(0, 0, 0, 0)
        self.topMenus = QFrame(self.contentSettings)
        self.topMenus.setObjectName(u"topMenus")
        self.topMenus.setFrameShape(QFrame.NoFrame)
        self.topMenus.setFrameShadow(QFrame.Raised)
        self.verticalLayout_14 = QVBoxLayout(self.topMenus)
        self.verticalLayout_14.setSpacing(0)
        self.verticalLayout_14.setObjectName(u"verticalLayout_14")
        self.verticalLayout_14.setContentsMargins(0, 0, 0, 0)

        self.verticalLayout_13.addWidget(self.topMenus, 0, Qt.AlignTop)


        self.verticalLayout_7.addWidget(self.contentSettings)


        self.horizontalLayout_4.addWidget(self.extraRightBox)


        self.verticalLayout_6.addWidget(self.content)

        self.bottomBar = QFrame(self.contentBottom)
        self.bottomBar.setObjectName(u"bottomBar")
        self.bottomBar.setMinimumSize(QSize(0, 22))
        self.bottomBar.setMaximumSize(QSize(16777215, 22))
        self.bottomBar.setFrameShape(QFrame.NoFrame)
        self.bottomBar.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_5 = QHBoxLayout(self.bottomBar)
        self.horizontalLayout_5.setSpacing(0)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.horizontalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.creditsLabel = QLabel(self.bottomBar)
        self.creditsLabel.setObjectName(u"creditsLabel")
        self.creditsLabel.setMaximumSize(QSize(16777215, 16))
        font5 = QFont()
        font5.setFamily(u"Segoe UI")
        font5.setBold(False)
        font5.setItalic(False)
        self.creditsLabel.setFont(font5)
        self.creditsLabel.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)

        self.horizontalLayout_5.addWidget(self.creditsLabel)

        self.version = QLabel(self.bottomBar)
        self.version.setObjectName(u"version")
        self.version.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.horizontalLayout_5.addWidget(self.version)

        self.frame_size_grip = QFrame(self.bottomBar)
        self.frame_size_grip.setObjectName(u"frame_size_grip")
        self.frame_size_grip.setMinimumSize(QSize(20, 0))
        self.frame_size_grip.setMaximumSize(QSize(20, 16777215))
        self.frame_size_grip.setFrameShape(QFrame.NoFrame)
        self.frame_size_grip.setFrameShadow(QFrame.Raised)

        self.horizontalLayout_5.addWidget(self.frame_size_grip)


        self.verticalLayout_6.addWidget(self.bottomBar)


        self.verticalLayout_2.addWidget(self.contentBottom)


        self.appLayout.addWidget(self.contentBox)


        self.appMargins.addWidget(self.bgApp)

        MainWindow.setCentralWidget(self.styleSheet)

        self.retranslateUi(MainWindow)

        self.stackedWidget.setCurrentIndex(2)
        self.stackedWidget_2.setCurrentIndex(0)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.titleLeftApp.setText(QCoreApplication.translate("MainWindow", u"누리 에이전트", None))
        self.titleLeftDescription.setText(QCoreApplication.translate("MainWindow", u"요양보호사 보조 누리봇", None))
        self.toggleButton.setText(QCoreApplication.translate("MainWindow", u"Hide", None))
        self.btn_home.setText(QCoreApplication.translate("MainWindow", u"DashBoard", None))
        self.btn_residents.setText(QCoreApplication.translate("MainWindow", u"Residents", None))
        self.btn_logs.setText(QCoreApplication.translate("MainWindow", u"Logs", None))
        self.btn_settings.setText(QCoreApplication.translate("MainWindow", u"Settings", None))

        self.titleRightInfo.setText(QCoreApplication.translate("MainWindow", u"PyDracula APP - Theme with colors based on Dracula for Python.", None))
#if QT_CONFIG(tooltip)
        self.minimizeAppBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Minimize", None))
#endif // QT_CONFIG(tooltip)
        self.minimizeAppBtn.setText("")
#if QT_CONFIG(tooltip)
        self.maximizeRestoreAppBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Maximize", None))
#endif // QT_CONFIG(tooltip)
        self.maximizeRestoreAppBtn.setText("")
#if QT_CONFIG(tooltip)
        self.closeAppBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Close", None))
#endif // QT_CONFIG(tooltip)
        self.closeAppBtn.setText("")
        self.lineEdit_2.setText("")
        self.lineEdit_2.setPlaceholderText(QCoreApplication.translate("MainWindow", "비밀번호를 입력해주세요."))

        self.resident_info.setTitle(QCoreApplication.translate("MainWindow", "입소자 정보"))
        self.label_2.setText(QCoreApplication.translate("MainWindow", "사진을 등록해주세요."))
        self.btn_face.setText(QCoreApplication.translate("MainWindow", "사진등록"))
        self.date_birth.setDisplayFormat(QCoreApplication.translate("MainWindow", "yyyy-MM-dd"))
        self.date_birth_2.setDisplayFormat(QCoreApplication.translate("MainWindow", "yyyy-MM-dd"))
        self.btn_face_2.setText(QCoreApplication.translate("MainWindow", "사진수정"))
        self.btn_discharge.setText(QCoreApplication.translate("MainWindow", "퇴소처리"))
        item = self.tbResidentList.horizontalHeaderItem(0)
        item.setText(QCoreApplication.translate("MainWindow", "이름"))
        item = self.tbResidentList.horizontalHeaderItem(1)
        item.setText(QCoreApplication.translate("MainWindow", "성별"))
        item = self.tbResidentList.horizontalHeaderItem(2)
        item.setText(QCoreApplication.translate("MainWindow", "생년월일"))
        item = self.tbResidentList.horizontalHeaderItem(3)
        item.setText(QCoreApplication.translate("MainWindow", "호실"))
        self.label_3.setText(QCoreApplication.translate("MainWindow", "이름 :"))
        self.btn_info.setText(QCoreApplication.translate("MainWindow", "신규등록"))
        self.btn_searchinfo.setText(QCoreApplication.translate("MainWindow", "검색"))
        self.btn_push.setText(QCoreApplication.translate("MainWindow", "등록"))
        self.check_discharge.setText(QCoreApplication.translate("MainWindow", "퇴소자 포함"))

        item = self.tableWidget.horizontalHeaderItem(0)
        item.setText(QCoreApplication.translate("MainWindow", "No."))
        item = self.tableWidget.horizontalHeaderItem(1)
        item.setText(QCoreApplication.translate("MainWindow", "이벤트 타입"))
        item = self.tableWidget.horizontalHeaderItem(2)
        item.setText(QCoreApplication.translate("MainWindow", "로봇ID"))
        item = self.tableWidget.horizontalHeaderItem(3)
        item.setText(QCoreApplication.translate("MainWindow", "로그 내용"))
        item = self.tableWidget.horizontalHeaderItem(4)
        item.setText(QCoreApplication.translate("MainWindow", "로그 시간"))
        self.tableWidget.setColumnWidth(0, 51)
        self.tableWidget.setColumnWidth(1, 150)
        self.tableWidget.setColumnWidth(2, 100)
        self.tableWidget.setColumnWidth(3, 330)
        self.tableWidget.setColumnWidth(4, 160)
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Fixed)
        self.btn_log_search.setText(QCoreApplication.translate("MainWindow", "검색"))
        self.dateEdit.setDisplayFormat(QCoreApplication.translate("MainWindow", "yyyy-MM-dd"))
        self.dateEdit_2.setDisplayFormat(QCoreApplication.translate("MainWindow", "yyyy-MM-dd"))
        # self.label_14.setText(QCoreApplication.translate("MainWindow", "~"))
        # self.label_15.setText(QCoreApplication.translate("MainWindow", "검색 기간 설정"))
        # self.label_16.setText(QCoreApplication.translate("MainWindow", "이벤트 타입"))
        # self.label_17.setText(QCoreApplication.translate("MainWindow", "로봇"))
        # self.label_18.setText(QCoreApplication.translate("MainWindow", "검색어"))

        # self.movie_label.setText(QCoreApplication.translate("MainWindow", u"Test", None))

        self.creditsLabel.setText(QCoreApplication.translate("MainWindow", u"By: Wanderson M. Pimenta", None))
        self.version.setText(QCoreApplication.translate("MainWindow", u"v1.0.3", None))

class RobotEntry(QWidget):
    clicked = Signal(int)  # 로봇 이름 시그널

    def __init__(self, robot: dict, parent=None):
        super().__init__(parent)
        self.robot_info = robot
        self.setObjectName(robot["name"])

        self.robot_layout = QVBoxLayout(self)
        self.robot_layout.setContentsMargins(5, 5, 5, 5)

        # 첫 줄: 이름만
        top_row = QHBoxLayout()
        name_label = QLabel(f"🤖 {robot['name']}")
        name_label.setStyleSheet("padding: 2px;")
        name_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        top_row.addWidget(name_label)
        self.robot_layout.addLayout(top_row)

        # 둘째 줄: 연결상태 + 배터리 + IP
        bottom_row = QHBoxLayout()
        status_label = QLabel("🟢 연결됨" if robot["status"] == "online" else "🔴 끊김")
        battery_label = QLabel(f"🔋 {robot['battery']}")

        for lbl in (status_label, battery_label):
            lbl.setStyleSheet("padding: 2px; margin-left: 5px; color: gray;")
            lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            bottom_row.addWidget(lbl)

        self.robot_layout.addLayout(bottom_row)
        self.setCursor(Qt.PointingHandCursor)

    def mousePressEvent(self, event):
        self.clicked.emit(self.robot_info["index"])
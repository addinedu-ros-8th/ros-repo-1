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
# from main import *
from modules import *

# WITH ACCESS TO MAIN WINDOW WIDGETS
# ///////////////////////////////////////////////////////////////
class AppFunctions():
    def setThemeHack(main_window):
        Settings.BTN_LEFT_BOX_COLOR = "background-color: #495474;"
        Settings.BTN_RIGHT_BOX_COLOR = "background-color: #495474;"
        Settings.MENU_SELECTED_STYLESHEET = MENU_SELECTED_STYLESHEET = """
        border-left: 22px solid qlineargradient(spread:pad, x1:0.034, y1:0, x2:0.216, y2:0, stop:0.499 rgba(255, 121, 198, 255), stop:0.5 rgba(85, 170, 255, 0));
        background-color: #566388;
        """

        ui = main_window.ui
        # SET MANUAL STYLES
        ui.map = MapDisplay(ui.label_display, u':/images/images/images/map_2.png')
        ui.btn_info.clicked.connect(lambda: UIFunctions.click_info(main_window))
        ui.btn_face.clicked.connect(lambda: UIFunctions.click_face(main_window))
        ui.btn_push.clicked.connect(lambda: UIFunctions.click_push(main_window))
        ui.tbResidentList.itemSelectionChanged.connect(lambda: UIFunctions.click_table(main_window))
        ui.btn_searchinfo.clicked.connect(lambda: UIFunctions.click_search(main_window))
        ui.btn_discharge.clicked.connect(lambda: UIFunctions.click_discharge(main_window))
        ui.btn_push_2.clicked.connect(lambda: UIFunctions.click_modify(main_window))
        ui.lineEdit.returnPressed.connect(lambda: UIFunctions.click_search(main_window))
        ui.btn_delete.clicked.connect(lambda: UIFunctions.click_delete(main_window))
        ui.refreshButton.clicked.connect(lambda: UIFunctions.click_refresh(main_window))
        ui.label_display.clicked.connect(lambda x, y: UIFunctions.click_map(main_window, x, y))
        ui.btn_log_search.clicked.connect(lambda: UIFunctions.click_log_search(main_window))
        ui.patrol_register_button.clicked.connect(lambda: UIFunctions.click_patrol_register(main_window))
        ui.patrol_cancel_button.clicked.connect(lambda: UIFunctions.click_patrol_cancel(main_window))
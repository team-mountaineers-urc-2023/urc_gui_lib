from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from rosgraph_msgs.msg import Log

from urc_gui_common.ui_python.ros_console_widget import Ui_RosConsoleWidget

# http://docs.ros.org/en/api/rosgraph_msgs/html/msg/Log.html
MSG_FMT = "[{name}]<{level}> {msg}"

LEVEL_TO_NAME = {
	1:  "DEBUG",
	2:  "INFO",
	4:  "WARN",
	8:  "ERROR",
	16: "FATAL",
}

class RosConsoleWidget(QWidget):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

		self.ui = Ui_RosConsoleWidget()
		self.ui.setupUi(self)

		self.ui.clear_button.clicked.connect(self.clear_console)

	def log_handler(self, log: Log):
		if log.level == log.DEBUG and self.ui.debug_checkbox.isChecked() \
		or log.level == log.INFO  and self.ui.info_checkbox.isChecked() \
		or log.level == log.WARN and self.ui.warn_checkbox.isChecked() \
		or log.level == log.ERROR and self.ui.error_checkbox.isChecked() \
		or log.level == log.FATAL and self.ui.fatal_checkbox.isChecked() \
		:
			self.display_log_message(log.name, log.level, log.msg)

	def display_log_message(self, name, level, msg):
		level_name = LEVEL_TO_NAME[level]
		self.ui.console_list.addItem(QListWidgetItem(MSG_FMT.format(name=name, level=level_name, msg=msg)))

	def clear_console(self):
		self.ui.console_list.clear()

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from urc_gui_common.ros_link import RosLink
from urc_gui_common.ui_python.tools_tab import Ui_Tools

class ToolsTab(QWidget):
	def __init__(self, roslink: RosLink, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.roslink = roslink

		self.ui = Ui_Tools()
		self.ui.setupUi(self)

		self.timer = self.ui.timer

		self.roslink.rosout.connect(self.ui.ros_console.log_handler)

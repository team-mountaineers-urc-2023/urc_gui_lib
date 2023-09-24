from PyQt5.QtWidgets import *

from urc_gui_common.ui_python.antenna_control_widget import Ui_AntennaControl

class AntennaControlWidget(QFrame):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

		self.ui = Ui_AntennaControl()
		self.ui.setupUi(self)

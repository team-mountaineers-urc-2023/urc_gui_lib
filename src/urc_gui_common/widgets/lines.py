from PyQt5.QtWidgets import *

class HLine(QFrame):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.setFrameShape(QFrame.HLine)
		self.setFrameShadow(QFrame.Sunken)

class VLine(QFrame):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.setFrameShape(QFrame.VLine)
		self.setFrameShadow(QFrame.Sunken)

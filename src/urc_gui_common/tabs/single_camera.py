#!/usr/bin/env python3.8

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from urc_gui_common.ros_link import RosLink
from urc_gui_common.widgets import SuperCameraWidget, SliderCompassWidget

class CameraTab(QWidget):
	def __init__(self, roslink: RosLink, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.roslink = roslink

		self.camera_widget = SuperCameraWidget()
		self.camera_widget.set_camera_funnel(self.roslink.camera_funnel)

		self.slider_compass = SliderCompassWidget(self)
		self.slider_compass.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Maximum)
		self.roslink.current_goal.connect(self.slider_compass.goal_handler)
		self.roslink.gps.connect(self.slider_compass.gps_handler)
		self.roslink.pose.connect(self.slider_compass.pose_handler)

		self.layout = QVBoxLayout()
		self.layout.addWidget(self.slider_compass)
		self.layout.addWidget(self.camera_widget)

		self.setLayout(self.layout)

#!/usr/bin/env python3.8

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from urc_gui_common.ros_link import RosLink
from urc_gui_common.widgets import SuperCameraWidget, SliderCompassWidget

class FourCameraTab(QWidget):
	def __init__(self, roslink: RosLink, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.roslink = roslink

		self.top_left_cam = SuperCameraWidget()
		self.top_right_cam = SuperCameraWidget()
		self.bot_left_cam = SuperCameraWidget()
		self.bot_right_cam = SuperCameraWidget()

		self.top_left_cam.set_camera_funnel(self.roslink.camera_funnel)
		self.top_right_cam.set_camera_funnel(self.roslink.camera_funnel)
		self.bot_left_cam.set_camera_funnel(self.roslink.camera_funnel)
		self.bot_right_cam.set_camera_funnel(self.roslink.camera_funnel)

		camera_layout = QGridLayout()
		camera_layout.setContentsMargins(0, 0, 0, 0)
		camera_layout.addWidget(self.top_left_cam, 0, 0)
		camera_layout.addWidget(self.top_right_cam, 0, 1)
		camera_layout.addWidget(self.bot_left_cam, 1, 0)
		camera_layout.addWidget(self.bot_right_cam, 1, 1)

		self.slider_compass = SliderCompassWidget(self)
		self.slider_compass.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Maximum)
		self.roslink.current_goal.connect(self.slider_compass.goal_handler)
		self.roslink.gps.connect(self.slider_compass.gps_handler)
		self.roslink.pose.connect(self.slider_compass.pose_handler)

		self.layout = QVBoxLayout()
		self.layout.addWidget(self.slider_compass)
		self.layout.addLayout(camera_layout)

		self.setLayout(self.layout)

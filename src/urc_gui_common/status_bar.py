from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from math import sqrt

from std_msgs.msg import String
from geometry_msgs.msg import Pose

from urc_gui_common.ros_link import RosLink
from urc_gui_common.widgets import VLine, TimerWidget

class StatusBar(QWidget):
		def __init__(self, *args, **kwargs):
			super().__init__(*args, **kwargs)

			### create widgets ###############################################

			self.battery_label = QLabel("Battery:")
			self.battery_display_label = QLabel("-")

			self.timer_label = QLabel("Countdown:")
			self.timer_display_label = QLabel("00:00")

			self.distance_label = QLabel("Distance:")
			self.distance_display_label = QLabel("- m")

			self.state_label = QLabel("State:")
			self.state_display_label = QLabel("-")

			self.planner_status_label = QLabel("Planner Status:")
			self.planner_status_display_label = QLabel("-")

			self.middle_spacer = QLabel("")

			### fill in layout ###############################################

			self.layout = QHBoxLayout()
			self.layout.setContentsMargins(0, 0, 0, 0)

			self.layout.addWidget(VLine(), 1)
			self.layout.addWidget(self.battery_label, 1, alignment=Qt.AlignLeft)
			self.layout.addWidget(self.battery_display_label, 2, alignment=Qt.AlignLeft)
			self.layout.addWidget(VLine(), 1)
			self.layout.addWidget(self.timer_label, 1)
			self.layout.addWidget(self.timer_display_label, 1)
			self.layout.addWidget(VLine(), 1)
			self.layout.addWidget(self.distance_label, 1)
			self.layout.addWidget(self.distance_display_label, 2)
			self.layout.addWidget(VLine(), 1)

			self.layout.addWidget(self.middle_spacer, 20)

			self.layout.addWidget(VLine(), 1)
			self.layout.addWidget(self.state_label, 1, alignment=Qt.AlignLeft)
			self.layout.addWidget(self.state_display_label, 5, alignment=Qt.AlignLeft)
			self.layout.addWidget(VLine(), 1)
			self.layout.addWidget(self.planner_status_label, 1, alignment=Qt.AlignLeft)
			self.layout.addWidget(self.planner_status_display_label, 3, alignment=Qt.AlignLeft)
			self.layout.addWidget(VLine(), 1)

			self.setLayout(self.layout)

			### end init #####################################################

		def connect_roslink(self, roslink: RosLink):
			roslink.pose.connect(self.update_distance)
			roslink.state.connect(self.update_state)
			roslink.planner_status.connect(self.update_planner_status)

		def connect_timer(self, timer: TimerWidget):
			timer.timer_signal.connect(self.update_timer)

		def update_timer(self, time: str):
			self.timer_display_label.setText(time)

		def update_distance(self, pose: Pose):
			dist = sqrt(pose.position.x**2 + pose.position.y**2)
			rounded_dist = round(dist)
			self.distance_display_label.setText(f'{rounded_dist} m')

		def update_state(self, state: String):
			self.state_display_label.setText(state.data)

		def update_planner_status(self, planner_status: String):
			self.planner_status_display_label.setText(planner_status.data)

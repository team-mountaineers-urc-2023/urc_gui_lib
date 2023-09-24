
from math import atan2, degrees

import pymap3d as pm

from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPoint

from urc_gui_common.widgets import MapPoint

### helpers ##################################################################

def clamp(value: float, lower: float, upper: float) -> float:
	return min(upper, max(value, lower))

def smaller_angle(angle):
	if angle > 180:
		return angle - 360
	elif angle < -180:
		return angle + 360
	return angle

### main #####################################################################

class SliderCompassWidget(QWidget):
	NORTH_OFFSET = 90

	def __init__(self, parent):
		QWidget.__init__(self, parent)

		self.rover_gps = None
		self.goal = None

		self.rover_angle = 0.0
		self.target_angle = 0.0

		self.min = -90
		self.max = 90

		self.slider = QSlider(Qt.Horizontal)
		self.slider.setMinimum(self.min)
		self.slider.setMaximum(self.max)
		self.slider.setStyleSheet("""
			QSlider::groove:horizontal {
				height: 4px;
				background: rgba(128, 128, 128, 64);
			}

			QSlider::handle:horizontal {
				background: rgb(255, 0, 0);
				border: 1px solid rgb(128, 0, 0);
				width: 12px;
				margin: -4.5px 0; /* handle is placed by default on the contents rect of the groove. Expand outside the groove */
				border-radius: 5px;
			}
		""")
		self.slider.setEnabled(False)

		self.center_slider = QSlider(Qt.Horizontal)
		self.center_slider.setMinimum(self.min)
		self.center_slider.setMaximum(self.max)
		self.center_slider.setStyleSheet("""
			QSlider::groove:horizontal {
				height: 0px;
				background: rgba(0, 0, 0, 0);
			}

			QSlider::handle:horizontal {
				background: rgb(192, 192, 192);
				border: 0px solid rgba(192, 192, 192, 0);
				width: 4px;
				margin: -10px 0; /* handle is placed by default on the contents rect of the groove. Expand outside the groove */
				border-radius: 0px;
			}
		""")
		self.center_slider.setEnabled(False)

		self.layout = QGridLayout()
		self.layout.setContentsMargins(0, 0, 0, 0)
		self.layout.addWidget(self.center_slider, 0, 0)
		self.layout.addWidget(self.slider, 0, 0)
		self.setLayout(self.layout)

	### handlers #############################################################

	def pose_handler(self, pose: Pose):
		orientation = pose.orientation
		orientation = QQuaternion(QVector4D(orientation.x, orientation.y, orientation.z, orientation.w))
		orientation = orientation.toEulerAngles()
		angle = (-orientation.z() + self.NORTH_OFFSET) % 360
		self.set_angle(angle)

	def point_at_goal(self):
		if self.rover_gps and self.goal:
			e, n, u = pm.geodetic2enu(
				self.goal.latitude,
				self.goal.longitude,
				0,
				self.rover_gps.latitude,
				self.rover_gps.longitude,
				0,
			)

			angle = degrees(atan2(e, n))
			self.set_target_angle(angle)

	def gps_handler(self, gps: GeoPoint):
		self.rover_gps = gps
		self.point_at_goal()

	def goal_handler(self, goal: MapPoint):
		self.goal = goal
		self.point_at_goal()

	### angle manipulation ###################################################

	def set_angle(self, angle):
		if angle != self.rover_angle:
			self.rover_angle = angle
			self.update()

	def set_target_angle(self, angle):
		if angle != self.target_angle:
			self.target_angle = angle
			self.update()

	### update slider ########################################################

	def update(self):
		value = 0

		if self.target_angle and self.rover_angle:
			difference = self.target_angle - self.rover_angle  # 0 - 360
			smaller_difference = smaller_angle(difference)  # -180 - 180
			value = clamp(smaller_difference, self.min, self.max)

		self.slider.setValue(value)

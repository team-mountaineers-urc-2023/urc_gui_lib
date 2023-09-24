from math import atan2, degrees

import pymap3d as pm

from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPoint

from urc_gui_common.widgets import MapPoint

NORTH_OFFSET = 90

class CompassWidget(QWidget):
	def __init__(self, parent):
		QWidget.__init__(self, parent)

		self.rover_gps = None
		self.goal = None

		self.rover_angle = 0.0
		self.target_angle = 0.0
		self._margins = 10
		self._pointText = {
			0: "N",
			45: "NE",
			90: "E",
			135: "SE",
			180: "S",
			225: "SW",
			270: "W",
			315: "NW" 
		}

	def paintEvent(self, event: QPaintEvent):
		painter = QPainter()
		painter.begin(self)
		painter.setRenderHint(QPainter.Antialiasing)
		self.draw_markings(painter)
		self.draw_needle(painter)
		self.draw_target(painter)
		painter.end()

	def draw_markings(self, painter: QPainter):
		painter.save()
		painter.translate(self.width() / 2, self.height() / 2)
		scale = min((self.width() - self._margins) / 120.0,
					(self.height() - self._margins) / 120.0)
		painter.scale(scale, scale)

		font = QFont(self.font())
		font.setPixelSize(10)
		metrics = QFontMetricsF(font)

		painter.setFont(font)
		painter.setPen(self.palette().color(QPalette.Shadow))

		i = 0
		while i < 360:
			if i % 45 == 0:
				painter.drawLine(0, -33, 0, -43)
				painter.drawText(-metrics.width(self._pointText[i]) / 2.0, -45, self._pointText[i])
			else:
				painter.drawLine(0, -38, 0, -43)

			painter.rotate(15)
			i += 15

		painter.restore()

	def draw_needle(self, painter: QPainter):
		painter.save()
		painter.translate(self.width() / 2, self.height() / 2)
		painter.rotate(self.rover_angle)
		scale = min((self.width() - self._margins) / 120.0,
					(self.height() - self._margins) / 120.0)
		painter.scale(scale, scale)

		painter.setPen(QPen(Qt.NoPen))
		painter.setBrush(self.palette().brush(QPalette.Shadow))

		painter.drawPolygon(
			QPolygon([
				QPoint(-5, 0), 
				QPoint(0, -38), 
				QPoint(5, 0),
				QPoint(0, 38),
				QPoint(-5, 0)
			])
		)

		painter.setBrush(self.palette().brush(QPalette.Highlight))

		painter.drawPolygon(
			QPolygon([
				QPoint(-5, 0),
				QPoint(0, -38),
				QPoint(5, 0),
				QPoint(-5, 0)
			])
		)

		painter.restore()

	def draw_target(self, painter: QPainter):
		painter.save()
		painter.translate(self.width() / 2, self.height() / 2)
		scale = min((self.width() - self._margins) / 120.0,
					(self.height() - self._margins) / 120.0)
		painter.scale(scale, scale)

		font = QFont(self.font())
		font.setPixelSize(10)
		metrics = QFontMetricsF(font)

		painter.setFont(font)
		painter.setPen(self.palette().color(QPalette.Highlight))

		painter.rotate(self.target_angle)

		painter.drawText(-metrics.width("<*>") / 2.0, -54, "<*>")

		painter.restore()

	def pose_handler(self, pose: Pose):
		orientation = pose.orientation
		orientation = QQuaternion(QVector4D(orientation.x, orientation.y, orientation.z, orientation.w))
		orientation = orientation.toEulerAngles()
		angle = (-orientation.z() + NORTH_OFFSET) % 360
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

	def set_angle(self, angle):
		if angle != self.rover_angle:
			self.rover_angle = angle
			self.update()

	def set_target_angle(self, angle):
		if angle != self.target_angle:
			self.target_angle = angle
			self.update()

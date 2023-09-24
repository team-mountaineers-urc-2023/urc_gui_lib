from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from urc_gui_common.helpers.validator import Validators, red_if_unacceptable

class AutonomyEditMarkerDialog(QDialog):
	def __init__(self, cur_row, cur_lat, cur_lon, cur_rad, cur_type, cur_id, cur_id_2):
		super().__init__()

		self.resize(399, 473)
		self.setWindowTitle("Alter Autonomy Marker")

		self.dialog_btns = QDialogButtonBox(self)
		self.dialog_btns.setGeometry(QRect(30, 430, 341, 32))
		self.dialog_btns.setOrientation(Qt.Horizontal)
		self.dialog_btns.setStandardButtons(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
		self.dialog_btns.accepted.connect(self.accept)
		self.dialog_btns.rejected.connect(self.reject)

		### Reorder Marker Options ###########################################

		self.reorder_btn = QRadioButton("Reorder Marker", self)
		self.reorder_btn.setGeometry(QRect(20, 20, 161, 27))
		self.reorder_btn.toggled.connect(self.reorder_toggled)

		self.line = QFrame(self)
		self.line.setGeometry(QRect(20, 40, 351, 20))
		self.line.setFrameShape(QFrame.HLine)
		self.line.setFrameShadow(QFrame.Sunken)

		self.row_label = QLabel("Row:", self)
		self.row_label.setGeometry(QRect(50, 60, 161, 21))
		self.row_entry = QLineEdit(self)
		self.row_entry.setPlaceholderText("Row")
		self.row_entry.setText(str(cur_row))
		self.row_entry.setValidator(Validators.row_validator)
		self.row_entry.textEdited.connect(lambda _: red_if_unacceptable(self.row_entry))
		self.row_entry.setGeometry(QRect(250, 60, 113, 29))
		self.row_entry.setEnabled(False)

		### Edit Marker Options ##############################################

		self.edit_btn = QRadioButton("Edit Marker", self)
		self.edit_btn.setGeometry(QRect(20, 100, 161, 27))
		self.edit_btn.toggled.connect(self.edit_toggled)

		self.line_2 = QFrame(self)
		self.line_2.setGeometry(QRect(20, 120, 351, 20))
		self.line_2.setFrameShape(QFrame.HLine)
		self.line_2.setFrameShadow(QFrame.Sunken)

		self.lat_label = QLabel("Latitude:", self)
		self.lat_label.setGeometry(QRect(50, 140, 161, 21))
		self.lat_entry = QLineEdit(self)
		self.lat_entry.setPlaceholderText("Latitude")
		self.lat_entry.setText(cur_lat)
		self.lat_entry.setValidator(Validators.latitude_validator)
		self.lat_entry.textEdited.connect(lambda _: red_if_unacceptable(self.lat_entry))
		self.lat_entry.setGeometry(QRect(250, 140, 113, 29))
		self.lat_entry.setEnabled(False)

		self.lon_label = QLabel("Longitude:", self)
		self.lon_label.setGeometry(QRect(50, 180, 161, 21))
		self.lon_entry = QLineEdit(self)
		self.lon_entry.setPlaceholderText("Longitude")
		self.lon_entry.setText(cur_lon)
		self.lon_entry.setValidator(Validators.longitude_validator)
		self.lon_entry.textEdited.connect(lambda _: red_if_unacceptable(self.lon_entry))
		self.lon_entry.setGeometry(QRect(250, 180, 113, 29))
		self.lon_entry.setEnabled(False)

		self.radius_label = QLabel("Radius (m):", self)
		self.radius_label.setGeometry(QRect(50, 220, 161, 21))
		self.radius_entry = QLineEdit(self)
		self.radius_entry.setPlaceholderText("Radius")
		self.radius_entry.setText(cur_rad)
		self.radius_entry.setValidator(Validators.radius_validator)
		self.radius_entry.textEdited.connect(lambda _: red_if_unacceptable(self.radius_entry))
		self.radius_entry.setGeometry(QRect(250, 220, 113, 29))
		self.radius_entry.setEnabled(False)

		self.type_label = QLabel("Marker Type:", self)
		self.type_label.setGeometry(QRect(50, 260, 161, 21))
		self.type_combo = QComboBox(self)
		self.type_combo.addItems(Validators.marker_types)
		self.type_combo.setCurrentIndex(Validators.marker_types.index(cur_type))
		self.type_combo.setGeometry(QRect(250, 260, 111, 29))
		self.type_combo.setEnabled(False)

		self.aruco_label = QLabel("Aruco ID:", self)
		self.aruco_label.setGeometry(QRect(50, 300, 161, 21))
		self.aruco_entry = QLineEdit(self)
		self.aruco_entry.setPlaceholderText("Aruco ID")
		self.aruco_entry.setText(cur_id)
		self.aruco_entry.setValidator(Validators.aruco_validator)
		self.aruco_entry.textEdited.connect(lambda _: red_if_unacceptable(self.aruco_entry))
		self.aruco_entry.setGeometry(QRect(250, 300, 113, 29))
		self.aruco_entry.setEnabled(False)

		self.aruco_2_label = QLabel("Aruco ID 2:", self)
		self.aruco_2_label.setGeometry(QRect(50, 340, 161, 21))
		self.aruco_2_entry = QLineEdit(self)
		self.aruco_2_entry.setPlaceholderText("Aruco ID 2")
		self.aruco_2_entry.setText(cur_id_2)
		self.aruco_2_entry.setValidator(Validators.aruco_validator)
		self.aruco_2_entry.textEdited.connect(lambda _: red_if_unacceptable(self.aruco_2_entry))
		self.aruco_2_entry.setGeometry(QRect(250, 340, 113, 29))
		self.aruco_2_entry.setEnabled(False)

		### Delete Button Options ############################################

		self.del_btn = QRadioButton("Delete Marker", self)
		self.del_btn.setGeometry(QRect(20, 380, 161, 27))
		self.del_btn.toggled.connect(self.del_toggled)

		self.line_3 = QFrame(self)
		self.line_3.setGeometry(QRect(20, 400, 351, 20))
		self.line_3.setFrameShape(QFrame.HLine)
		self.line_3.setFrameShadow(QFrame.Sunken)

	def reorder_toggled(self):
		if self.reorder_btn.isChecked():
			self.row_entry.setEnabled(True)
		else:
			self.row_entry.setEnabled(False)

	def edit_toggled(self):
		if self.edit_btn.isChecked():
			self.lat_entry.setEnabled(True)
			self.lon_entry.setEnabled(True)
			self.radius_entry.setEnabled(True)
			self.type_combo.setEnabled(True)
			self.aruco_entry.setEnabled(True)
			self.aruco_2_entry.setEnabled(True)
		else:
			self.lat_entry.setEnabled(False)
			self.lon_entry.setEnabled(False)
			self.radius_entry.setEnabled(False)
			self.type_combo.setEnabled(False)
			self.aruco_entry.setEnabled(False)
			self.aruco_2_entry.setEnabled(False)

	def del_toggled(self):
		pass

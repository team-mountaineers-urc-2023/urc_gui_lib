from typing import List

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

class Validators:
	row_validator = QIntValidator(0, 100)
	latitude_validator = QDoubleValidator(-90.0, 90.0, -1)
	longitude_validator = QDoubleValidator(-180.0, 180.0, -1)
	radius_validator = QDoubleValidator(0, 10000, -1)
	marker_types = ["no_marker", "one_marker", "gate", "intermediary_marker"]
	aruco_validator = QIntValidator(0, 99)

def red_if_unacceptable(line_edit: QLineEdit):
	if line_edit.hasAcceptableInput():
		line_edit.setStyleSheet("")
		return False
	else:
		line_edit.setStyleSheet("background-color: #ff6060")
		return True

def valid_entries(entries: List[QLineEdit]):
	valid = True
	for entry in entries:
		if red_if_unacceptable(entry):
			valid = False
	return valid

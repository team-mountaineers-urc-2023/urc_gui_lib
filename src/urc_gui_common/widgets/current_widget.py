from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from urc_gui_common.ui_python.current_widget import Ui_CurrentWidget

_slider_stylesheet = """
QSlider::groove:vertical {{
	border: 2px solid hsl({hue}, 255, {dark_lum});
	background: white;
	width: 10px;
	border-radius: 4px;
}}

QSlider::add-page:vertical {{
	background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 hsl({hue}, 255, {dark_lum}), stop: 1 hsl({hue}, 255, {bright_lum}));
	border: 4px solid rgba(0, 0, 0, 0);
	width: 10px;
	border-radius: 4px;
}}
"""

class CurrentWidget(QWidget):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

		self.precision = 1
		self.min_i = 0
		self.max_i = 1

		self.ui = Ui_CurrentWidget()
		self.ui.setupUi(self)

		self.ui.slider.setEnabled(False)
		self.set_current_range(self.min_i, self.max_i, self.precision)
		self.set_current(.5)

		self.ui.slider.valueChanged.connect(self.update_color)

	def set_current_range(self, min_f: float, max_f: float, precision: int = 0):
		self.precision = precision
		self.min_i = int(min_f * 10**precision)
		self.max_i = int(max_f * 10**precision)
		self.ui.slider.setRange(self.min_i, self.max_i)

	def set_current(self, currentf: float):
		current = int(currentf * 10**self.precision)
		self.ui.slider.setSliderPosition(current)

	def update_color(self, position: int):
		fullness = (position - self.min_i) / (self.max_i - self.min_i)
		hue_shift = int(120 * fullness)
		lum_shift = int(32 * (.5 - abs(.5 - fullness)) * 2)
		hue = 120 - hue_shift
		dark_lum = 160 + lum_shift
		bright_lum = 223 + lum_shift
		self.ui.slider.setStyleSheet(_slider_stylesheet.format(hue=hue, dark_lum=dark_lum, bright_lum=bright_lum))

from collections import namedtuple
from enum import Enum
from typing import List, Dict

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import pyqtgraph as pg
from pyqtgraph.graphicsItems.PlotDataItem import PlotDataItem

from urc_gui_common.ui_python.graph import Ui_GraphWidget

Color = namedtuple('Color', 'r g b')

class Colors(Color, Enum):
	BLACK = Color(0, 0, 0)
	RED = Color(255, 0, 0)
	ORANGE = Color(255, 128, 0)
	YELLOW = Color(240, 200, 70)
	GREEN = Color(0, 192, 0)
	BLUE = Color(0, 0, 255)
	PURPLE = Color(255, 0, 255)

class GraphWidget(QWidget):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

		self.lines: Dict[PlotDataItem] = {}

		self.ui = Ui_GraphWidget()
		self.ui.setupUi(self)

		self.ui.autoscale_button.clicked.connect(self.autoscale)
		self.ui.default_scale_button.clicked.connect(self.default_scale)

		self.proxy = pg.SignalProxy(self.ui.graph.scene().sigMouseMoved, rateLimit=30, slot=self.mouse_moved)

	def setup_graph(self, title: str, x_label: str, y_label: str, x_range: range, y_range: range, foregound: str='k', background: str='w'):
		self.label_styles = {'color':foregound}
		self.x_range = x_range
		self.y_range = y_range
		self.default_scale()

		# set labels
		self.ui.graph.setTitle(title, color=foregound)
		self.ui.graph.setLabel('bottom', x_label, **self.label_styles)
		self.ui.graph.setLabel('left', y_label, **self.label_styles)

		# set background
		self.ui.graph.setBackground(background)
		self.ui.graph.showGrid(x=True, y=True)

		# hide buttons
		self.ui.graph.hideButtons()

	def plot(self, x_data: List[float], y_data: List[float], name: str, color: Color=Colors.BLACK, width: int=3):
		self.line_pen = pg.mkPen(color=color, width=width)
		if name in self.lines:
			self.lines[name].setData(x_data, y_data)
		else:
			self.lines[name] = self.ui.graph.plot(x_data, y_data, pen=self.line_pen)

	def clear(self, name: str):
		if name in self.lines:
			self.plot([], [], name)

	def default_scale(self):
		self.ui.graph.setXRange(self.x_range.start, self.x_range.stop)
		self.ui.graph.setYRange(self.y_range.start, self.y_range.stop)

	def autoscale(self):
		self.ui.graph.enableAutoRange(axis='x')
		self.ui.graph.enableAutoRange(axis='y')
		self.ui.graph.setAutoVisible(x=True)
		self.ui.graph.setAutoVisible(y=True)

	def mouse_moved(self, event):
		position = event[0]
		if self.ui.graph.sceneBoundingRect().contains(position):
			mouse_point = self.ui.graph.getPlotItem().vb.mapSceneToView(position)
			x = round(mouse_point.x(), 2)
			y = round(mouse_point.y(), 2)
			self.ui.x_label.setText(f'x: {x}')
			self.ui.y_label.setText(f'y: {y}')

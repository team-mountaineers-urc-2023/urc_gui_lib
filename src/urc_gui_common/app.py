#!/usr/bin/env python3.8

import sys
import signal
from pathlib import Path

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from urc_gui_common.status_bar import StatusBar

class Window(QMainWindow):
	def __init__(self, title: str="WVU URC", *args, **kwargs):
		super().__init__(*args, **kwargs)
		
		icon_path = Path(__file__).parent.resolve() / "resources/flying_wv3.png"
		self.setWindowIcon(QIcon(str(icon_path)))
		self.setWindowTitle(title)

		self.tabs = QTabWidget()

		self.splitter = QSplitter()
		self.splitter.addWidget(self.tabs)
		self.splitter.setCollapsible(0, False)  # prevent collapse of tabs
		self.splitter.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
		self.splitter.setHandleWidth(0)

		self.status_bar = StatusBar()

		self.layout = QVBoxLayout()
		self.layout.addWidget(self.splitter)
		self.layout.addWidget(self.status_bar)

		self.main_widget = QWidget()
		self.main_widget.setLayout(self.layout)
		self.setCentralWidget(self.main_widget)

		self.shortcut_fullscreen = QShortcut(QKeySequence('F11'), self)
		self.shortcut_fullscreen.activated.connect(self.toggle_fullscreen)

		self.shortcut_exit_fullscreen = QShortcut(QKeySequence('Escape'), self)
		self.shortcut_exit_fullscreen.activated.connect(self.exit_fullscreen)

		self.shortcut_quit = QShortcut(QKeySequence('Ctrl+Q'), self)
		self.shortcut_quit.activated.connect(self.sigHandler)

	def toggle_fullscreen(self):
		if self.isFullScreen():
			self.showNormal()
		else:
			self.showFullScreen()

	def exit_fullscreen(self):
		self.showNormal()

	# Handler for when ctrl-c is hit
	def sigHandler(*args):
		QApplication.quit()

class AppRunner():
	def __init__(self, WindowType: QMainWindow):
		self.WindowType = WindowType

	def start(self):
		app = QApplication(sys.argv)

		window = self.WindowType()
		window.show()

		# Add signal to catch ctrl-c
		signal.signal(signal.SIGINT, window.sigHandler)
		# Signal only works if python code is running but QT is c++
		# We add a timer to periodically run python code
		timer = QTimer()
		timer.start(500) # Tick every 500 ms
		timer.timeout.connect(lambda: None)

		sys.exit(app.exec_())

def main():
	app = AppRunner(Window)
	app.start()

if __name__ == '__main__':
	main()

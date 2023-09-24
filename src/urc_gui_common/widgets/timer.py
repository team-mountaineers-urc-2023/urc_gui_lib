from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from urc_gui_common.ui_python.set_time_dialog import Ui_timer_dialog
from urc_gui_common.ui_python.timer_widget import Ui_Timer

# Converts seconds to minutes to display timer properly
def secs_to_minsec(secs):
	mins = secs // 60
	secs = secs % 60
	minsec = f'{mins:02}:{secs:02}'
	return minsec

# Converts minsec to seconds
def minsec_to_secs(mins, secs):
	total_secs = (mins * 60) + secs
	return total_secs

class TimerWidget(QWidget):
	timer_signal = pyqtSignal(str)

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		
		self.timer_signal.connect(self.update_display)

		# Whether timer is going or not
		self.start = False

		# Start timer with 0 seconds left
		self.duration_sec = 0

		# Setup the timer widget
		timer_widget = Ui_Timer()
		timer_widget.setupUi(self)

		# Where the time is shown
		self.timer_label = timer_widget.timer_label

		# Button that sets the timer
		self.set_button = timer_widget.set_button
		self.set_button.clicked.connect(self.set_timer)

		# Button that starts the timer
		self.start_button = timer_widget.start_button
		self.start_button.clicked.connect(self.start_timer)
		
		# Button that pauses the timer
		self.pause_button = timer_widget.pause_button
		self.pause_button.clicked.connect(self.pause_timer)

		# Button that resets the timer to 00:00
		self.reset_button = timer_widget.reset_button
		self.reset_button.clicked.connect(self.reset_timer)

		# Button that subtracts 1 second from the timer
		self.subtract1_button = timer_widget.minus1_button
		self.subtract1_button.clicked.connect(lambda: self.subtract_time(1))

		# Button that subtracts 30 seconds from the timer
		self.subtract30_button = timer_widget.minus30_button
		self.subtract30_button.clicked.connect(lambda: self.subtract_time(30))

		# Button that subtracts 60 seconds from the timer
		self.subtract60_button = timer_widget.minus60_button
		self.subtract60_button.clicked.connect(lambda: self.subtract_time(60))

		# Button that adds 1 second to the timer
		self.add1_button = timer_widget.plus1_button
		self.add1_button.clicked.connect(lambda: self.add_time(1))

		# Button that adds 30 seconds to the timer
		self.add30_button = timer_widget.plus30_button
		self.add30_button.clicked.connect(lambda: self.add_time(30))

		# Button that adds 60 second to the timer
		self.add60_button = timer_widget.plus60_button
		self.add60_button.clicked.connect(lambda: self.add_time(60))
		
		# Update the countdown time
		timer = QTimer(self)
		timer.timeout.connect(self.update_time)
		timer.start(1000)

	# Sets the timer for countdown
	def set_timer(self):
		# Make start flag false
		self.start = False

		# Get countdown time
		dialog_box = TimerDialogBox()
		if dialog_box.exec():
			minutes_text = dialog_box.minutes_edit.text()
			seconds_text = dialog_box.seconds_edit.text()

			minutes = int(minutes_text) if minutes_text else 0
			seconds = int(seconds_text) if seconds_text else 0

			self.duration_sec = minsec_to_secs(minutes, seconds)
			minsec = secs_to_minsec(self.duration_sec)
			self.timer_signal.emit(minsec)

	# Starts the timer
	def start_timer(self):
		self.start = True

		if self.duration_sec == 0:
			self.start = False
	
	# Pauses the timer
	def pause_timer(self):
		self.start = False
	
	# Resets the timer
	def reset_timer(self):
		# Reset timer values
		self.start = False
		self.duration_sec = 0
		self.timer_signal.emit("00:00")
	
	def subtract_time(self, seconds: int):
		if self.duration_sec - seconds < 0:
			self.reset_timer()
			return
		
		self.duration_sec -= seconds
		minsec = secs_to_minsec(self.duration_sec)
		self.timer_signal.emit(minsec)
	
	def add_time(self, seconds: int):
		self.duration_sec += seconds
		minsec = secs_to_minsec(self.duration_sec)
		self.timer_signal.emit(minsec)
	
	# Updates the gui with the updated countdown time
	def update_time(self):
		if self.start:
			self.duration_sec -= 1

			if self.duration_sec == 0:
				self.start = False
				self.timer_signal.emit("00:00")
				return

			minsec = secs_to_minsec(self.duration_sec)
			self.timer_signal.emit(minsec)

	def update_display(self, minsec):
		self.timer_label.setText(minsec)

class TimerDialogBox(QDialog):
	def __init__(self):
		super().__init__()

		ui = Ui_timer_dialog()
		ui.setupUi(self)

		self.minutes_edit = ui.minutes_edit
		self.seconds_edit = ui.seconds_edit

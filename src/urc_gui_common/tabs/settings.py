#!/usr/bin/env python3.8

import re
import os
from collections import defaultdict
from dataclasses import dataclass
from typing import List

import yaml
from yaml.representer import Representer
safe_dumper = yaml.SafeDumper
safe_dumper.add_representer(defaultdict, Representer.represent_dict)

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import rospy

from std_msgs.msg import String

from theora_webcams.srv import GetResolutions, ChangeVideo, ChangeVideoResponse
from theora_webcams.msg import Resolution, Framerate

from urc_gui_common.ros_link import RosLink
from urc_gui_common.widgets import SuperCameraWidget, HLine
from urc_gui_common.helpers.file_helper import file_basenames

gui_presets_dir = os.path.expanduser('~/.ros/gui_presets')

@dataclass
class CameraInfo:
	name: str
	alias: str
	connected: bool
	subscribed: bool
	flip: bool
	theora_webcam: bool
	image_source: str
	current_width: int
	current_height: int
	current_fps: str
	resolutions: List[Resolution]
	change_video: rospy.ServiceProxy

class SettingsTab(QWidget):
	def __init__(self, roslink: RosLink, super_camera_widgets: List[SuperCameraWidget] = None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.roslink = roslink
		self.super_camera_widgets = super_camera_widgets if super_camera_widgets else []
		self.current_camera = -1

		self.camera_info: List[CameraInfo] = []

		# warn on autonomy if no cameras running
		self.roslink.marker_list.connect(self.autonomy_input_check)  # markers input
		self.roslink.state.connect(self.autonomy_started_check)  # autonomy started
		self.state = ''

		# setup data
		self.camera_name_label = QLabel("Name: ")
		self.camera_name = QLabel("camera_name")

		self.alias_name_label = QLabel("Alias: ")
		self.alias_name = QLabel("alias")

		self.quality_label = QLabel("Quality: ")
		self.quality = QLabel("quality")

		self.status_label = QLabel("Status: ")
		self.status = QLabel("status")

		self.flip_label = QLabel("Flip Camera: ")
		self.flip = QCheckBox()
		self.flip.clicked.connect(self.camera_flip_changed)

		self.camera_res_label = QLabel("Resolution: ")
		self.camera_resolutions = QComboBox()
		self.camera_resolutions.currentIndexChanged.connect(self.camera_res_selection_changed)

		self.camera_fps_label = QLabel("Framerate: ")
		self.camera_framerates = QComboBox()
		self.camera_framerates.currentIndexChanged.connect(self.camera_fps_selection_changed)

		self.camera_commit_but = QPushButton("Commit changes")
		self.camera_commit_but.pressed.connect(self.camera_commit_pressed)
		self.camera_commit_but.setEnabled(False)

		self.camera_stop_but = QPushButton("Stop Camera")
		self.camera_stop_but.pressed.connect(self.camera_stop_pressed)

		# setup list
		self.camera_list = QTableWidget()
		self.camera_list.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
		self.camera_list.setColumnCount(1)
		self.camera_list.setHorizontalHeaderLabels(["Camera"])

		self.camera_list.cellChanged.connect(self.update_edited_camera_alias)
		self.camera_list.cellClicked.connect(self.camera_selection_changed)
		self.camera_list.currentCellChanged.connect(self.camera_selection_changed)

		self.scan_timer = QTimer()
		self.scan_timer.timeout.connect(self.refresh_cameras)
		self.scan_timer.start(1000)

		# put things together
		self.camera_settings = QWidget()
		settings_layout = QGridLayout()
		settings_layout.setAlignment(Qt.AlignTop)

		preset_90p = QPushButton("160x90 10fps")
		preset_90p.pressed.connect(lambda:self.set_cur_camera_resolution(160, 90, 10, 1))

		preset_180p = QPushButton("320x180 10fps")
		preset_180p.pressed.connect(lambda:self.set_cur_camera_resolution(320, 180, 10, 1))

		preset_360p = QPushButton("640x360 10fps")
		preset_360p.pressed.connect(lambda:self.set_cur_camera_resolution(640, 360, 10, 1))

		preset_480p = QPushButton("864x480 10fps")
		preset_480p.pressed.connect(lambda:self.set_cur_camera_resolution(864, 480, 10, 1))

		preset_720p = QPushButton("1280x720 10fps")
		preset_720p.pressed.connect(lambda:self.set_cur_camera_resolution(1280, 720, 10, 1))

		preset_896p = QPushButton("1600x896 10fps")
		preset_896p.pressed.connect(lambda:self.set_cur_camera_resolution(1600, 896, 10, 1))

		preset_1080p = QPushButton("1920x1080 5fps")
		preset_1080p.pressed.connect(lambda:self.set_cur_camera_resolution(1920, 1080, 5, 1))

		resolution_presets = QHBoxLayout()
		resolution_presets.addWidget(preset_90p)
		resolution_presets.addWidget(preset_180p)
		resolution_presets.addWidget(preset_360p)
		resolution_presets.addWidget(preset_480p)
		resolution_presets.addWidget(preset_720p)
		resolution_presets.addWidget(preset_896p)
		resolution_presets.addWidget(preset_1080p)

		hline = HLine()

		cameras_preset_label = QLabel("Camera Presets:")

		self.cameras_save_line_edit = QLineEdit()

		cameras_save_but = QPushButton("Save")
		cameras_save_but.pressed.connect(self.save_camera_info)

		self.cameras_load_combo = QComboBox()
		self.cameras_load_combo.clear()
		self.cameras_load_combo.addItems(file_basenames(gui_presets_dir))

		cameras_load_but = QPushButton("Load")
		cameras_load_but.pressed.connect(self.load_camera_info)

		cameras_del_but = QPushButton("Delete")
		cameras_del_but.pressed.connect(self.delete_camera_info)

		cameras_preset = QGridLayout()
		cameras_preset.addWidget(self.cameras_save_line_edit, 0, 0, 1, 1)
		cameras_preset.addWidget(cameras_save_but, 1, 0, 1, 1)
		cameras_preset.addWidget(self.cameras_load_combo, 0, 1, 1, 2)
		cameras_preset.addWidget(cameras_load_but, 1, 1, 1, 1)
		cameras_preset.addWidget(cameras_del_but, 1, 2, 1, 1)
		cameras_preset.setColumnStretch(0, 2)
		cameras_preset.setColumnStretch(1, 1)
		cameras_preset.setColumnStretch(2, 1)

		settings_layout.addWidget(self.camera_name_label, 	0,  0, Qt.AlignRight)
		settings_layout.addWidget(self.camera_name, 		0,  1, Qt.AlignLeft)
		settings_layout.addWidget(self.alias_name_label, 	1,  0, Qt.AlignRight)
		settings_layout.addWidget(self.alias_name, 			1,  1, Qt.AlignLeft)
		settings_layout.addWidget(self.quality_label, 		2,  0, Qt.AlignRight)
		settings_layout.addWidget(self.quality, 			2,  1, Qt.AlignLeft)
		settings_layout.addWidget(self.status_label, 		3,  0, Qt.AlignRight)
		settings_layout.addWidget(self.status, 				3,  1, Qt.AlignLeft)
		settings_layout.addWidget(self.flip_label, 			4,  0, Qt.AlignRight)
		settings_layout.addWidget(self.flip, 				4,  1, Qt.AlignLeft)
		settings_layout.addWidget(self.camera_res_label, 	5,  0, Qt.AlignRight)
		settings_layout.addWidget(self.camera_resolutions, 	5,  1, Qt.AlignLeft)
		settings_layout.addWidget(self.camera_fps_label, 	6,  0, Qt.AlignRight)
		settings_layout.addWidget(self.camera_framerates, 	6,  1, Qt.AlignLeft)
		settings_layout.addLayout(resolution_presets, 		7,  0, 1, 2)
		settings_layout.addWidget(self.camera_commit_but, 	8,  0, 1, 2)
		settings_layout.addWidget(self.camera_stop_but, 	9,  0, 1, 2)
		settings_layout.addWidget(hline, 					10, 0, 1, 2)
		settings_layout.addWidget(cameras_preset_label, 	11, 0, 1, 2)
		settings_layout.addLayout(cameras_preset, 			12, 0, 1, 2)

		self.camera_settings.setLayout(settings_layout)

		layout = QGridLayout()
		layout.addWidget(self.camera_list, 0, 0, Qt.AlignLeft)
		layout.addWidget(self.camera_settings, 0, 1)
		layout.setColumnStretch(0, 0)
		layout.setColumnStretch(1, 1)

		self.setLayout(layout)

	def search_for_cameras(self):
		# First go through and assume all are not connected
		for camera in self.camera_info:
			camera.connected = False

		# Now go through and find all the connected cameras
		camera_topic_re = r"^/cameras/([^/]+)/image_raw$"
		found_cameras = []
		for topic in rospy.get_published_topics():
			if match := re.match(camera_topic_re, topic[0]):
				name = match.group(1)
				found_cameras.append(name)

		# Finally go through the known connected cameras and either add them to
		# our list or set them to connected
		new_cameras = []
		for name in found_cameras:
			new_camera = True

			# check if we have the camera already
			for cam in self.camera_info:
				if cam.name == name:
					cam.connected = True
					new_camera = False

			if new_camera:
				# Get the resolutions and make the camera info
				get_resolutions = rospy.ServiceProxy(f'/cameras/{name}/get_resolutions', GetResolutions)
				change_video = rospy.ServiceProxy(f'/cameras/{name}/change_video', ChangeVideo)

				try:
					resolutions = get_resolutions().resolutions
				except rospy.service.ServiceException:
					resolutions = []

				self.camera_info.append(CameraInfo(name, name, True, False, False, False, None, '?', '?', '?', resolutions, change_video))
				new_cameras.append(name)

		# update camera table with new cameras
		if new_cameras:
			self.camera_list.setRowCount(len(self.camera_info))

			for i,cam in enumerate(self.camera_info):
				self.camera_list.setItem(i, 0, QTableWidgetItem(cam.alias))

			self.update_camera_funnel()

			if self.camera_list.currentRow() == -1:
				self.camera_list.setCurrentCell(0, 0)

		return found_cameras
	
	def update_camera_funnel(self):
		self.roslink.camera_funnel.set_cameras(
			[cam.name for cam in self.camera_info],
			[cam.alias for cam in self.camera_info],
			[cam.change_video for cam in self.camera_info]
		)
	
	def update_camera_info(self):
		# update camera info using camera subscriber
		for cam_info in self.camera_info:
			cam_sub = self.roslink.camera_funnel.subscribers.get(cam_info.name, None)
			if cam_sub:
				cam_info.theora_webcam = cam_sub.theora_webcam
				cam_info.current_height = cam_sub.height
				cam_info.current_width = cam_sub.width
				cam_info.current_fps = f'{cam_sub.framerate_num}/{cam_sub.framerate_den}'
				cam_info.subscribed = cam_sub.subscribed
				cam_info.image_source = cam_sub.image_source
			else:
				cam_info.theora_webcam = False
				cam_info.current_height = '?'
				cam_info.current_width = '?'
				cam_info.current_fps = '?/1'
				cam_info.subscribed = False
				cam_info.image_source = None

		# update current camera info display
		self.camera_selection_changed(self.camera_list.currentRow(), 0, 0, 0, refresh_status=False, changed_camera=False)

	def refresh_cameras(self):
		self.search_for_cameras()
		self.update_camera_info()

	def camera_flip_changed(self, val):
		if self.current_camera != -1:
			self.set_camera_flip(self.current_camera, bool(val))

	def set_camera_flip(self, camera_index, val: bool):
		if self.current_camera == camera_index and val:
			self.flip.setChecked(True)
		self.camera_info[camera_index].flip = val
		self.roslink.camera_funnel.flip[self.camera_info[camera_index].name] = val

	def camera_selection_changed(self, cur_row, cur_col=0, prev_row=0, prev_col=0, refresh_status=True, changed_camera=True):
		valid_row = 0 <= cur_row < len(self.camera_info)
		if not valid_row:
			return

		cur_cam = self.camera_info[cur_row]
		self.camera_name.setText(cur_cam.name)
		self.alias_name.setText(cur_cam.alias)
		self.flip.setChecked(cur_cam.flip)

		if refresh_status:
			if not cur_cam.connected:
				self.status.setText("Not Connected")
			elif not cur_cam.subscribed:
				self.status.setText("Connected, Not Subscribed")
			else:
				self.status.setText(f"Connected, Subscribed to {cur_cam.image_source}")

		if changed_camera:
			res_items = ['']
			for res in self.camera_info[cur_row].resolutions:
				res_items.append(f'{res.width}x{res.height}')

			self.camera_resolutions.clear()
			self.camera_resolutions.addItems(res_items)

		self.quality.setText(f'{cur_cam.current_width}x{cur_cam.current_height} {cur_cam.current_fps}fps')

		self.current_camera = cur_row

	def update_edited_camera_alias(self, row, col):
		# When a cell is edited, we update the camera alias
		new_alias = self.camera_list.item(row, col).text()
		self.update_camera_alias(row, new_alias, update_alias=False)

	def update_camera_alias(self, camera_index, alias, update_alias=True):
		if update_alias:
			self.camera_list.item(camera_index, 0).setText(alias)
		self.camera_info[camera_index].alias = alias
		if self.current_camera == camera_index:
			self.alias_name.setText(self.camera_info[camera_index].alias)

		self.update_camera_funnel()

	def set_cur_camera_resolution(self, width, height, fps_num, fps_den, start=True):
		# if no camera selected, dont try and set the resolution
		if self.current_camera == -1:
			QMessageBox(
				QMessageBox.Critical,
				"No Camera Selected",
				f"Could not set camera to {width}x{height} @ {fps_num}/{fps_den} fps",
				QMessageBox.Ok,
			).exec()
			self.status.setText("No Camera Selected :(")
			return

		self.set_camera_resolution(self.current_camera, width, height, fps_num, fps_den, start)

	def set_camera_resolution(self, camera_index, width, height, fps_num, fps_den, start=True, force_restart=False):
		cam = self.camera_info[camera_index]
		try:
			result: ChangeVideoResponse = cam.change_video(width, height, fps_num, fps_den, start, force_restart)
		except rospy.service.ServiceException:
			result = None

		if result and result.success:
			self.status.setText("Successfully changed video :)")

		else:
			QMessageBox(
				QMessageBox.Critical,
				"Failed to set camera",
				f"Could not set camera to {width}x{height} @ {fps_num}/{fps_den} fps",
				QMessageBox.Ok,
			).exec()

			self.status.setText("Failed to change video :(")

	def camera_commit_pressed(self):
		cur_cam = self.camera_info[self.current_camera]
		chosen_res: Resolution = cur_cam.resolutions[self.camera_resolutions.currentIndex() - 1]
		chosen_fps: Framerate = chosen_res.framerates[self.camera_framerates.currentIndex() - 1]
		self.set_cur_camera_resolution(
			chosen_res.width, chosen_res.height,
			chosen_fps.numerator, chosen_fps.denominator
		)

	def camera_stop_pressed(self):
		self.set_cur_camera_resolution(0, 0, 0, 0, False)

	def camera_res_selection_changed(self, i):
		self.camera_framerates.clear()

		# the first one is blank
		if i <= 0 or self.current_camera == -1:
			return

		# so offset the rest
		i -= 1

		fps_items = ['']
		for fps in self.camera_info[self.current_camera].resolutions[i].framerates:
			fps_items.append(f'{fps.numerator}/{fps.denominator}')

		self.camera_framerates.addItems(fps_items)

	def camera_fps_selection_changed(self, i):
		# Only allow to click the button if we aren't on the blank fps
		self.camera_commit_but.setEnabled(i > 0)

	### presets ##############################################################

	def save_camera_info(self):
		# get filename to save camera info to
		filename = self.cameras_save_line_edit.text()
		if not filename:
			filename = 'unnamed'

		# determine which SuperCameraWidget is subscribed to which camera
		camera_to_widget = {}
		for super_camera_widget_index, super_camera_widget in enumerate(self.super_camera_widgets):
			selected_camera_alias = super_camera_widget.selector.currentText()
			if selected_camera_alias:
				camera_to_widget[selected_camera_alias] = super_camera_widget_index

		# store the camera info in a dictionary of dictionaries
		preset_data = defaultdict(dict)
		for camera in self.camera_info:
			preset_data[camera.name]['theora_webcam'] = camera.theora_webcam
			preset_data[camera.name]['alias'] = camera.alias
			preset_data[camera.name]['flip'] = camera.flip
			if camera.alias in camera_to_widget:
				preset_data[camera.name]['widget_no'] = camera_to_widget[camera.alias]
			if camera.theora_webcam and camera.connected:
				preset_data[camera.name]['width'] = camera.current_width
				preset_data[camera.name]['height'] = camera.current_height
				preset_data[camera.name]['fps'] = camera.current_fps

		# write to file
		if not os.path.exists(gui_presets_dir):
			os.makedirs(gui_presets_dir)
		filepath = f'{gui_presets_dir}/{filename}.yaml'
		with open(filepath, 'w') as preset_file:
			yaml.safe_dump(preset_data, preset_file)

		# confirm to user that file has been saved
		QMessageBox(
			QMessageBox.Information,
			f"Camera Info Saved",
			f"Saved camera info to file {filepath}.",
			QMessageBox.Ok,
		).exec()

		# refresh load_combo
		self.cameras_load_combo.clear()
		self.cameras_load_combo.addItems(file_basenames(gui_presets_dir))

	def load_camera_info(self):
		# check if a file has been specified
		filename = self.cameras_load_combo.currentText()
		if not filename:
			QMessageBox(
				QMessageBox.Critical,
				"No filename selected",
				"Set a filename to load presets from.",
				QMessageBox.Ok,
			).exec()
			return

		# check if file exists
		filepath = f'{gui_presets_dir}/{filename}.yaml'
		if not os.path.exists(filepath):
			QMessageBox(
				QMessageBox.Critical,
				"File does not exist",
				f"File {filepath} does not exist.",
				QMessageBox.Ok,
			).exec()
			return

		# read camera info from file
		with open(filepath, 'r') as preset_file:
			preset_data = yaml.safe_load(preset_file)

		preload_camera_index = self.current_camera

		# load camera data
		failed_cameras = []
		for camera_name in preset_data.keys():
			try:
				theora_webcam = bool(preset_data[camera_name]['theora_webcam'])
				alias = str(preset_data[camera_name]['alias'])
				flipped = bool(preset_data[camera_name]['flip'])
				widget_no = int(preset_data[camera_name].get('widget_no', -1))
				super_camera_widget = self.super_camera_widgets[widget_no] if len(self.super_camera_widgets) > widget_no >= 0 else None
			except KeyError:
				failed_cameras.append((camera_name, f'Invalid config parameters (theora_webcam/alias/flipped/widget) for camera {camera_name}'))
				continue

			for index, camera in enumerate(self.camera_info):
				if camera.name == camera_name:
					self.update_camera_alias(index, alias)
					self.set_camera_flip(index, flipped)
					if super_camera_widget:
						super_camera_widget.selector.setCurrentIndex(index + 1)  # skip the blank option
					if theora_webcam and camera.connected:
						try:
							width = int(preset_data[camera_name]['width'])
							height = int(preset_data[camera_name]['height'])
							fps_num_str, fps_den_str = preset_data[camera_name]['fps'].split('/')
							fps_num, fps_den = int(fps_num_str), int(fps_den_str)
							start = width and height and fps_num and fps_den
							self.set_camera_resolution(index, width, height, fps_num, fps_den, start, False)
						except (KeyError, ValueError):
							failed_cameras.append((camera_name, f'Invalid start parameters (width/height/fps) for camera {camera_name}'))

		# reset current camera to before the load happened
		self.camera_selection_changed(preload_camera_index, 0, 0, 0, refresh_status=False, changed_camera=True)

		failed_camera_summary = ''
		for camera, msg in failed_cameras:
			failed_camera_summary += '\n * ' + msg

		# confirm to user that file has been loaded
		QMessageBox(
			QMessageBox.Information,
			"Camera Info Loaded",
			f"Loaded camera info from file {filepath}. {failed_camera_summary}",
			QMessageBox.Ok,
		).exec()

	def delete_camera_info(self):
		# check if a file has been specified
		filename = self.cameras_load_combo.currentText()
		if not filename:
			QMessageBox(
				QMessageBox.Critical,
				"No filename selected",
				"Set a filename to delete.",
				QMessageBox.Ok,
			).exec()
			return

		# check if file exists
		filepath = f'{gui_presets_dir}/{filename}.yaml'
		if not os.path.exists(filepath):
			QMessageBox(
				QMessageBox.Critical,
				"File does not exist",
				f"File {filepath} does not exist.",
				QMessageBox.Ok,
			).exec()
			return

		# ask user to confirm deleting file
		confirmation_box = QMessageBox(
			QMessageBox.Question,
			f"Delete preset?",
			f"Are you sure you want to delete preset {filename}?",
			QMessageBox.Yes | QMessageBox.No,
		)
		
		# delete file, with confirmation
		if confirmation_box.exec() == QMessageBox.Yes:
			os.remove(filepath)

			# refresh load_combo
			self.cameras_load_combo.clear()
			self.cameras_load_combo.addItems(file_basenames(gui_presets_dir))

	### warn if using autonomy with no cameras ###############################

	def autonomy_check(self):

		for camera_info in self.camera_info:
			if camera_info.current_width and camera_info.current_height:
				return

		QMessageBox(
			QMessageBox.Critical,
			"Cameras not started",
			f"Autonomy use detected, but no cameras were started.",
			QMessageBox.Ok,
		).exec()

	def autonomy_input_check(self, _):
		self.autonomy_check()

	def autonomy_started_check(self, state: String):
		state_str = state.data
		if state_str != self.state and state_str == 'Loiter':
			self.state = state_str
			self.autonomy_check()

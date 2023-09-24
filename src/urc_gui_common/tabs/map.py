#!/usr/bin/env python3.8

import os
import time
import threading

import pymap3d as pm
import yaml

os.environ['QT_API'] = 'pyqt5'
from pyqtlet2.leaflet import *

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import rospy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Pose

from aruco_finder.msg import FoundMarkerList
from marker_interfacing.msg import GeodeticMarker, GeodeticMarkerList

from urc_gui_common.ros_link import RosLink
from urc_gui_common.widgets import MapPoint, MapViewer, HLine
from urc_gui_common.helpers.file_helper import file_basenames
from urc_gui_common.helpers.internet import can_access_internet
from urc_gui_common.helpers.validator import Validators, red_if_unacceptable, valid_entries
from urc_gui_common.helpers.route_optimization import optimize_route, algorithms, cost_functions
from urc_gui_common.ui_python.autonomy_edit_dialog import AutonomyEditMarkerDialog
from urc_gui_common.ui_python.optimization_dialog import Ui_optimization_dialog
from urc_gui_common import tile_scraper

### prepare yaml #############################################################

def geodetic_marker_list_representer(dumper: yaml.SafeDumper, markers: GeodeticMarkerList) -> yaml.nodes.MappingNode:
	return dumper.represent_mapping("!GeodeticMarkerList", {
		"markers": markers.markers,
		"count": markers.count,
	})

def geodetic_marker_representer(dumper: yaml.SafeDumper, marker: GeodeticMarker) -> yaml.nodes.MappingNode:
	return dumper.represent_mapping("!GeodeticMarker", {
		"gps": marker.gps,
		"waypoint_error": marker.waypoint_error,
		"marker_type": marker.marker_type,
		"aruco_id": marker.aruco_id,
		"aruco_id_2": marker.aruco_id_2,
	})

def geopoint_representer(dumper: yaml.SafeDumper, gp: GeoPoint) -> yaml.nodes.MappingNode:
	return dumper.represent_mapping("!GeoPoint", {
		"latitude": gp.latitude,
		"longitude": gp.longitude,
		"altitude": gp.altitude,
	})

safe_dumper = yaml.SafeDumper
safe_dumper.add_representer(GeodeticMarkerList, geodetic_marker_list_representer)
safe_dumper.add_representer(GeodeticMarker, geodetic_marker_representer)
safe_dumper.add_representer(GeoPoint, geopoint_representer)

def geodetic_marker_list_constructor(loader: yaml.SafeLoader, node: yaml.nodes.MappingNode) -> GeodeticMarkerList:
	return GeodeticMarkerList(**loader.construct_mapping(node))

def geodetic_marker_constructor(loader: yaml.SafeLoader, node: yaml.nodes.MappingNode) -> GeodeticMarker:
	return GeodeticMarker(**loader.construct_mapping(node))

def geopoint_constructor(loader: yaml.SafeLoader, node: yaml.nodes.MappingNode) -> GeoPoint:
	return GeoPoint(**loader.construct_mapping(node))

loader = yaml.SafeLoader
loader.add_constructor("!GeodeticMarkerList", geodetic_marker_list_constructor)
loader.add_constructor("!GeodeticMarker", geodetic_marker_constructor)
loader.add_constructor("!GeoPoint", geopoint_constructor)

##############################################################################

autonomy_saves_dir = os.path.expanduser('~/.ros/autonomy_saves')

def clamp(value: float, lower: float, upper: float) -> float:
	return min(upper, max(value, lower))

class MapTab(QWidget):
	def __init__(self, roslink: RosLink, *args, **kwargs):
		super().__init__(*args, **kwargs)

		### override functions to subclass ###

		self.map_viewer: MapViewer = self.init_map_viewer()
		self.markers_tab: QTabWidget = self.init_markers_tab()
		self.setLayout(self.init_layout())
		self.init_map()

		######################################

		self.last_map_click = 0
		self.roslink = roslink
		self.roslink.gps.connect(self.gps_handler)
		self.roslink.pose.connect(self.pose_handler)
		self.roslink.marker_list.connect(self.aut_marker_list_handler)
		self.roslink.found_marker_list.connect(self.found_marker_list_handler)

	def init_map_viewer(self):
		map_viewer = MapViewer()
		map_viewer.map.clicked.connect(lambda l: self.map_click_handler(l['latlng']))
		map_viewer.add_point_layer('Autonomy', 'blue', 'green', 'yellow')
		return map_viewer

	def init_markers_tab(self):
		self.init_aut_marker_input()
		self.init_found_marker_output()
		markers_tab = QTabWidget()
		markers_tab.addTab(self.aut_marker_input, "Autonomy")
		markers_tab.addTab(self.found_marker_output, "Found Markers")
		return markers_tab

	def init_layout(self):
		map_settings = QHBoxLayout()
		map_server_label = QLabel("MapServer:")
		self.map_server_choices = QComboBox()
		self.map_server_choices.addItems(tile_scraper.MapServers.names())
		self.map_server_choices.currentIndexChanged.connect(self.update_map_server)
		map_settings.addWidget(map_server_label, 1)
		map_settings.addWidget(self.map_server_choices, 3)

		left_pane = QWidget()
		left_pane_layout = QVBoxLayout()
		left_pane_layout.addWidget(self.markers_tab)
		left_pane_layout.addLayout(map_settings)
		left_pane.setLayout(left_pane_layout)

		splitter = QSplitter()
		splitter.addWidget(left_pane)
		splitter.addWidget(self.map_viewer)
		splitter.setStretchFactor(1, 1)  # make map stretch to fill available space

		layout = QGridLayout()
		layout.addWidget(splitter)
		return layout

	def init_map(self):
		# pre-cache map tiles if possible
		if can_access_internet():
			# cache tiles for later, since there is internet now
			t = threading.Thread(target=tile_scraper.main)
			t.start()

			initial_map_server = tile_scraper.MapServers.ARCGIS_World_Imagery
		else:
			initial_map_server = tile_scraper.MapServers.ARCGIS_World_Imagery_Cache

		self.select_map_server(initial_map_server)

	def init_aut_marker_input(self):
		self.autosave_enabled = False
		self.marker_ids = []
		self.marker_list = GeodeticMarkerList()
		self.selected_row = None
		self.default_selected_row = -1

		self.aut_table = QTableWidget()
		self.aut_table.setMinimumWidth(150)
		self.aut_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
		self.aut_table.setColumnCount(6)
		self.aut_table.setHorizontalHeaderLabels(["Lat", "Lon", "Radius", "Type", "ID", "ID 2"])
		self.aut_table.setEditTriggers(QTableWidget.NoEditTriggers)
		self.aut_table.setSelectionBehavior(QTableWidget.SelectRows)
		self.aut_table.cellClicked.connect(self.aut_marker_select_handler)
		self.aut_table.currentCellChanged.connect(lambda cr, cc, pr, pc: self.aut_marker_select_handler(cr, cc))
		self.aut_table.cellDoubleClicked.connect(self.aut_marker_alter_handler)

		self.aut_lat_entry = QLineEdit()
		self.aut_lat_entry.setPlaceholderText("Latitude")
		self.aut_lat_entry.setValidator(Validators.latitude_validator)
		self.aut_lat_entry.textEdited.connect(lambda _: red_if_unacceptable(self.aut_lat_entry))

		self.aut_lon_entry = QLineEdit()
		self.aut_lon_entry.setPlaceholderText("Longitude")
		self.aut_lon_entry.setValidator(Validators.longitude_validator)
		self.aut_lon_entry.textEdited.connect(lambda _: red_if_unacceptable(self.aut_lon_entry))

		self.aut_radius_entry = QLineEdit()
		self.aut_radius_entry.setPlaceholderText("Radius (m)")
		self.aut_radius_entry.setValidator(Validators.radius_validator)
		self.aut_radius_entry.textEdited.connect(lambda _: red_if_unacceptable(self.aut_radius_entry))

		self.aut_marker_type = QComboBox()
		self.aut_marker_type.addItems(Validators.marker_types)

		self.aut_aruco_id_entry = QLineEdit()
		self.aut_aruco_id_entry.setPlaceholderText("Aruco ID")
		self.aut_aruco_id_entry.setValidator(Validators.aruco_validator)
		self.aut_aruco_id_entry.textEdited.connect(lambda _: red_if_unacceptable(self.aut_aruco_id_entry))

		self.aut_aruco_id_2_entry = QLineEdit()
		self.aut_aruco_id_2_entry.setPlaceholderText("Aruco ID 2")
		self.aut_aruco_id_2_entry.setValidator(Validators.aruco_validator)
		self.aut_aruco_id_2_entry.textEdited.connect(lambda _: red_if_unacceptable(self.aut_aruco_id_2_entry))

		self.aruco_id_layout = QHBoxLayout()
		self.aruco_id_layout.addWidget(self.aut_aruco_id_entry)
		self.aruco_id_layout.addWidget(self.aut_aruco_id_2_entry)

		hline1 = HLine()

		self.aut_insert_button = QPushButton("Insert Marker")
		self.aut_insert_button.clicked.connect(self.insert_aut_marker)

		self.aut_add_button = QPushButton("Add Marker")
		self.aut_add_button.clicked.connect(self.add_aut_marker)

		self.new_marker_layout = QHBoxLayout()
		self.new_marker_layout.addWidget(self.aut_insert_button)
		self.new_marker_layout.addWidget(self.aut_add_button)

		self.aut_clear_button = QPushButton("Clear Markers")
		self.aut_clear_button.clicked.connect(self.clear_aut_markers)

		self.aut_optimize_button = QPushButton("Optimize Route")
		self.aut_optimize_button.clicked.connect(self.optimize_route_handler)

		hline2 = HLine()

		self.autonomy_save_line_edit = QLineEdit()

		autonomy_save_but = QPushButton("Save")
		autonomy_save_but.pressed.connect(self.save_autonomy_markers)

		self.autonomy_load_combo = QComboBox()
		self.autonomy_load_combo.clear()
		self.autonomy_load_combo.addItems(file_basenames(autonomy_saves_dir))

		autonomy_load_but = QPushButton("Load")
		autonomy_load_but.pressed.connect(self.load_autonomy_markers)

		autonomy_del_but = QPushButton("Delete")
		autonomy_del_but.pressed.connect(self.delete_autonomy_markers)

		autonomy_saves = QGridLayout()
		autonomy_saves.addWidget(self.autonomy_save_line_edit, 0, 0, 1, 1)
		autonomy_saves.addWidget(autonomy_save_but, 1, 0, 1, 1)
		autonomy_saves.addWidget(self.autonomy_load_combo, 0, 1, 1, 2)
		autonomy_saves.addWidget(autonomy_load_but, 1, 1, 1, 1)
		autonomy_saves.addWidget(autonomy_del_but, 1, 2, 1, 1)
		autonomy_saves.setColumnStretch(0, 2)
		autonomy_saves.setColumnStretch(1, 1)
		autonomy_saves.setColumnStretch(2, 1)

		layout = QVBoxLayout()
		layout.setContentsMargins(0, 0, 0, 0)
		layout.addWidget(self.aut_table)
		layout.addWidget(self.aut_lat_entry)
		layout.addWidget(self.aut_lon_entry)
		layout.addWidget(self.aut_radius_entry)
		layout.addWidget(self.aut_marker_type)
		layout.addLayout(self.aruco_id_layout)
		layout.addWidget(hline1)
		layout.addLayout(self.new_marker_layout)
		layout.addWidget(self.aut_clear_button)
		layout.addWidget(self.aut_optimize_button)
		layout.addWidget(hline2)
		layout.addLayout(autonomy_saves)

		self.aut_marker_input = QWidget()
		self.aut_marker_input.setLayout(layout)

	def init_found_marker_output(self):
		self.found_marker_table = QTableWidget()
		self.found_marker_table.setMinimumWidth(150)
		self.found_marker_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
		self.found_marker_table.setColumnCount(3)
		self.found_marker_table.setHorizontalHeaderLabels(["ID", "Lat", "Lon"])
		self.found_marker_table.setEditTriggers(QTableWidget.NoEditTriggers)
		self.found_marker_table.setSelectionBehavior(QTableWidget.SelectRows)

		self.found_marker_clear_button = QPushButton("Clear Found Markers")
		self.found_marker_clear_button.clicked.connect(self.clear_found_markers)

		layout = QVBoxLayout()
		layout.setContentsMargins(0, 0, 0, 0)
		layout.addWidget(self.found_marker_table)
		layout.addWidget(self.found_marker_clear_button)

		self.found_marker_output = QWidget()
		self.found_marker_output.setLayout(layout)

	def update_map_server(self, map_server_index: int):
		map_server = tile_scraper.MapServers.values()[map_server_index]
		self.map_viewer.set_map_server(map_server.tile_url, map_server.layer_count)

	def select_map_server(self, map_server: tile_scraper.MapServer):
		try:
			map_server_index = tile_scraper.MapServers.values().index(map_server)
		except ValueError:
			err_msg = f'Could not set map server because it is not available in tile_scraper.MapServers\n'\
			f'Available MapServers: {tile_scraper.MapServers.names()}\n'\
			f'Selected MapServer: {map_server.name}'
			rospy.logerr(err_msg)
			return

		if self.map_server_choices.currentIndex() == map_server_index:
			self.update_map_server(map_server_index)
		else:
			self.map_server_choices.setCurrentIndex(map_server_index)

	def aut_marker_select_handler(self, row, col):
		self.map_viewer.set_highlighted_point('Autonomy', row)
		self.selected_row = row
		if self.marker_list.markers:
			m = self.marker_list.markers[0]
			self.roslink.current_goal.emit(MapPoint(m.gps.latitude, m.gps.longitude, m.waypoint_error, ''))

	def aut_marker_alter_handler(self, row, column):
		def failed_to_alter_marker():
			# TODO: popup box
			rospy.logwarn('Failed to alter marker!')

		# prepare popup box
		cur_lat = self.aut_table.item(row, 0).text()
		cur_lon = self.aut_table.item(row, 1).text()
		cur_rad = self.aut_table.item(row, 2).text()
		cur_type = self.aut_table.item(row, 3).text()
		cur_id = self.aut_table.item(row, 4).text()
		cur_id_2 = self.aut_table.item(row, 5).text()
		dialog_box = AutonomyEditMarkerDialog(row + 1, cur_lat, cur_lon, cur_rad, cur_type, cur_id, cur_id_2)

		# handle user request
		if dialog_box.exec():
			marker_id = self.marker_ids[row]

			# user requested the marker be reordered
			if dialog_box.reorder_btn.isChecked():
				target_row = int(dialog_box.row_entry.text()) - 1
				row_count = len(self.marker_ids) - 1
				new_row = clamp(target_row, 0, row_count)
				new_following_marker_id = self.marker_ids[new_row] # id of the marker the selected marker should be reordered before

				# set row to select when marker list comes back
				self.default_selected_row = new_row

				# send request to reorder marker
				try:
					self.roslink.reorder_marker(marker_id, new_following_marker_id)
				except rospy.service.ServiceException as e:
					failed_to_alter_marker()
					rospy.logerr(e)

			# user requested the marker have its values edited
			elif dialog_box.edit_btn.isChecked():
				# validate entry inputs
				entries = [dialog_box.lat_entry, dialog_box.lon_entry, dialog_box.radius_entry, dialog_box.aruco_entry, dialog_box.aruco_2_entry]
				if not valid_entries(entries):
					failed_to_alter_marker()
					return

				lat = float(dialog_box.lat_entry.text())
				lon = float(dialog_box.lon_entry.text())
				radius = float(dialog_box.radius_entry.text())
				marker_type = dialog_box.type_combo.currentText()
				aruco_id = int(dialog_box.aruco_entry.text())
				aruco_id_2 = int(dialog_box.aruco_2_entry.text())

				# set row to select when marker list comes back
				self.default_selected_row = row

				# send request to edit marker
				try:
					self.roslink.edit_marker(lat, lon, 0.0, radius, marker_type, aruco_id, aruco_id_2, marker_id)
				except rospy.service.ServiceException as e:
					failed_to_alter_marker()
					rospy.logerr(e)

			# user request the marker be deleted
			elif dialog_box.del_btn.isChecked():
				# set row to select when marker list comes back
				self.default_selected_row = row - 1

				try:
					self.roslink.remove_marker(marker_id)
				except rospy.service.ServiceException as e:
					failed_to_alter_marker()
					rospy.logerr(e)

	def gps_handler(self, gps: GeoPoint):
		self.map_viewer.set_robot_position(gps.latitude, gps.longitude)
	
	def pose_handler(self, pose: Pose):
		orientation = pose.orientation
		orientation = QQuaternion(QVector4D(orientation.x, orientation.y, orientation.z, orientation.w))
		orientation = orientation.toEulerAngles()
		self.map_viewer.set_robot_rotation(-orientation.z())

	def aut_marker_list_handler(self, marker_list: GeodeticMarkerList):
		self.disp_aut_markers(marker_list)
		if self.marker_list.markers:
			m = self.marker_list.markers[0]
			self.roslink.current_goal.emit(MapPoint(m.gps.latitude, m.gps.longitude, m.waypoint_error, ''))
		if self.autosave_enabled:
			self.save_autonomy_markers_to_file('autosave')
			self.autosave_enabled = False

	def disp_aut_markers(self, marker_list: GeodeticMarkerList):
		# clear current markers
		self.map_viewer.unhighlight_point('Autonomy')
		marker_count = len(marker_list.markers)
		self.aut_table.setRowCount(marker_count)
		self.marker_ids = [0] * marker_count  # clear marker_ids

		# store marker_list
		self.marker_list = marker_list

		# populate markers in table
		for row, marker in enumerate(marker_list.markers):
			marker: GeodeticMarker
			items = [marker.gps.latitude, marker.gps.longitude, marker.waypoint_error, marker.marker_type, marker.aruco_id, marker.aruco_id_2]
			for col, s in enumerate(items):
				self.aut_table.setItem(row, col, QTableWidgetItem(str(s)))
			self.marker_ids[row] = marker.marker_id
		
		# display markers
		points = [MapPoint(m.gps.latitude, m.gps.longitude, m.waypoint_error, '') for m in marker_list.markers]
		self.map_viewer.set_points('Autonomy', points)

		# highlight latest marker
		row_to_highlight = self.default_selected_row
		if 0 <= row_to_highlight < len(marker_list.markers):
			self.aut_table.setCurrentCell(row_to_highlight, 0)
			self.map_viewer.set_highlighted_point('Autonomy', row_to_highlight)
		else:
			self.map_viewer.unhighlight_point('Autonomy')

	def map_click_handler(self, latlng):
		now = time.time()
		if now - self.last_map_click < 0.500:
			self.aut_lat_entry.setText(f"{latlng['lat']:.7f}")
			self.aut_lon_entry.setText(f"{latlng['lng']:.7f}")
		
		self.last_map_click = now

	def get_marker_values(self):
		no_marker = self.aut_marker_type.currentIndex() == 0
		one_marker = self.aut_marker_type.currentIndex() == 1
		gate_marker = self.aut_marker_type.currentIndex() == 2
		intermediary_marker = self.aut_marker_type.currentIndex() == 3

		# if no_marker or intermediary_marker type and no entries, use defaults
		if no_marker or intermediary_marker:
			if self.aut_radius_entry.text() == "":
				self.aut_radius_entry.setText("0")
			if self.aut_aruco_id_entry.text() == "":
				self.aut_aruco_id_entry.setText("99")

		# if not gate_marker, use defaults for secondary aruco id
		if not gate_marker:
			if self.aut_aruco_id_2_entry.text() == "":
				self.aut_aruco_id_2_entry.setText("99")

		# validate entry inputs
		entries = [self.aut_lat_entry, self.aut_lon_entry, self.aut_radius_entry, self.aut_aruco_id_entry, self.aut_aruco_id_2_entry]
		if not valid_entries(entries):
			raise ValueError

		# get marker values
		latf = float(self.aut_lat_entry.text())
		lonf = float(self.aut_lon_entry.text())
		altf = 0.0
		radf = float(self.aut_radius_entry.text())
		marker = self.aut_marker_type.currentText()
		aruco_id = int(self.aut_aruco_id_entry.text())
		aruco_id_2 = int(self.aut_aruco_id_2_entry.text())

		# reset input values
		self.aut_lat_entry.clear()
		self.aut_lon_entry.clear()
		self.aut_marker_type.setCurrentIndex(3) if intermediary_marker else self.aut_marker_type.setCurrentIndex(0)
		self.aut_radius_entry.clear()
		self.aut_aruco_id_entry.clear()
		self.aut_aruco_id_2_entry.clear()

		return latf, lonf, altf, radf, marker, aruco_id, aruco_id_2

	def add_aut_marker(self):
		def failed_to_add_marker():
			self.aut_add_button.setStyleSheet("background-color: #ffff00")
			self.aut_add_button.setText("Add Marker (Failed)")

		try:
			latf, lonf, altf, radf, marker, aruco_id, aruco_id_2 = self.get_marker_values()

			# set row to select when marker list comes back (set to next row)
			self.default_selected_row = len(self.marker_ids)

			try:
				self.autosave_enabled = True
				self.roslink.add_marker(latf, lonf, altf, radf, marker, aruco_id, aruco_id_2)
				self.aut_add_button.setStyleSheet("")
				self.aut_add_button.setText("Add Marker")
			except rospy.service.ServiceException as e:
				failed_to_add_marker()
				rospy.logerr(e)
		except ValueError:
			failed_to_add_marker()

	def insert_aut_marker(self):
		def failed_to_insert_marker():
			self.aut_insert_button.setStyleSheet("background-color: #ffff00")
			self.aut_insert_button.setText("Insert Marker (Failed)")

		try:
			latf, lonf, altf, radf, marker, aruco_id, aruco_id_2 = self.get_marker_values()

			# get selected row, if any
			selected_row = self.selected_row if self.selected_row is not None else len(self.marker_ids)

			# set row to select when marker list comes back (set to next row)
			self.default_selected_row = selected_row

			# find id of selected marker (marker to insert before)
			new_following_marker_id = self.marker_ids[selected_row] if 0 <= selected_row < len(self.marker_ids) else 0

			try:
				self.autosave_enabled = True
				self.roslink.insert_marker(latf, lonf, altf, radf, marker, aruco_id, aruco_id_2, new_following_marker_id)
				self.aut_insert_button.setStyleSheet("")
				self.aut_insert_button.setText("Insert Marker")
			except rospy.service.ServiceException as e:
				failed_to_insert_marker()
				rospy.logerr(e)
		except ValueError:
			failed_to_insert_marker()

	def failed_to_clear_markers(self):
		self.aut_clear_button.setStyleSheet("background-color: #ffff00")
		self.aut_clear_button.setText("Clear Markers (Failed)")

	def clear_aut_markers(self):
		# ask user to confirm clearing markers
		confirmation_box = QMessageBox(
			QMessageBox.Question,
			f"Clear markers?",
			f"Are you sure you want to clear all markers?",
			QMessageBox.Yes | QMessageBox.No,
		)

		# clear markers, with confirmation
		if confirmation_box.exec() == QMessageBox.Yes:
			try:
				self.default_selected_row = -1
				self.roslink.clear_markers()
				self.aut_clear_button.setStyleSheet("")
				self.aut_clear_button.setText("Clear Markers")
			except rospy.service.ServiceException as e:
				self.failed_to_clear_markers()
				rospy.logerr(e)

	def save_autonomy_markers(self):
		# get filename to save autonomy info to
		filename = self.autonomy_save_line_edit.text()
		if not filename:
			filename = 'unnamed'

		filepath = self.save_autonomy_markers_to_file(filename)

		# confirm to user that file has been saved
		QMessageBox(
			QMessageBox.Information,
			f"Autonomy Marker Info Saved",
			f"Saved autonomy marker info to file {filepath}.",
			QMessageBox.Ok,
		).exec()

	def save_autonomy_markers_to_file(self, filename):
		# write to file
		if not os.path.exists(autonomy_saves_dir):
			os.makedirs(autonomy_saves_dir)
		filepath = f'{autonomy_saves_dir}/{filename}.yaml'
		with open(filepath, 'w') as save_file:
			yaml.safe_dump(self.marker_list, save_file)

		# refresh load_combo
		self.autonomy_load_combo.clear()
		self.autonomy_load_combo.addItems(file_basenames(autonomy_saves_dir))

		return filepath

	def load_autonomy_markers(self):
		# check if a file has been specified
		filename = self.autonomy_load_combo.currentText()
		if not filename:
			QMessageBox(
				QMessageBox.Critical,
				"No filename selected",
				"Set a filename to load presets from.",
				QMessageBox.Ok,
			).exec()
			return

		# check if file exists
		filepath = f'{autonomy_saves_dir}/{filename}.yaml'
		if not os.path.exists(filepath):
			QMessageBox(
				QMessageBox.Critical,
				"File does not exist",
				f"File {filepath} does not exist.",
				QMessageBox.Ok,
			).exec()
			return

		# read autonomy marker info from file
		with open(filepath, 'r') as save_file:
			marker_list = yaml.safe_load(save_file)

		# load autonomy data
		for marker in marker_list.markers:
			try:
				self.roslink.add_marker(marker.gps.latitude, marker.gps.longitude, marker.gps.altitude, marker.waypoint_error, marker.marker_type, marker.aruco_id, marker.aruco_id_2)
			except rospy.service.ServiceException as e:
				self.failed_to_add_marker()
				rospy.logerr(e)

		# confirm to user that file has been loaded
		QMessageBox(
			QMessageBox.Information,
			"Autonomy Marker Info Loaded",
			f"Loaded autonomy marker info from file {filepath}.",
			QMessageBox.Ok,
		).exec()

	def delete_autonomy_markers(self):
		# check if a file has been specified
		filename = self.autonomy_load_combo.currentText()
		if not filename:
			QMessageBox(
				QMessageBox.Critical,
				"No filename selected",
				"Set a filename to delete.",
				QMessageBox.Ok,
			).exec()
			return

		# check if file exists
		filepath = f'{autonomy_saves_dir}/{filename}.yaml'
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
			self.autonomy_load_combo.clear()
			self.autonomy_load_combo.addItems(file_basenames(autonomy_saves_dir))

### aruco markers ############################################################

	def found_marker_list_handler(self, marker_list: FoundMarkerList):
		self.found_marker_table.setRowCount(len(marker_list.markers))
		for row, marker in enumerate(marker_list.markers):

			# get enu information of marker
			aruco_id = marker.aruco_id
			x = marker.marker_enu.x
			y = marker.marker_enu.y
			z = marker.marker_enu.z

			# calculate lat/long position of marker
			lat0 = self.roslink.global_origin.latitude
			lon0 = self.roslink.global_origin.longitude
			h0 = self.roslink.global_origin.altitude
			lat, lon, h = pm.enu2geodetic(x, y, z, lat0, lon0, h0)

			items = [aruco_id, lat, lon]
			for col, s in enumerate(items):
				self.found_marker_table.setItem(row, col, QTableWidgetItem(str(s)))

			if aruco_id in self.map_viewer.aruco_markers:
				self.map_viewer.aruco_markers[aruco_id].setIcon(self.map_viewer.aruco_icon)
				self.map_viewer.set_marker_position(aruco_id, lat, lon)
			else:
				position = [lat, lon]
				aruco_marker = L.marker(position)
				aruco_marker.bindTooltip(f'Aruco ID: {aruco_id}')
				aruco_marker.setIcon(self.map_viewer.aruco_icon)
				self.map_viewer.aruco_markers_layer.addLayer(aruco_marker)
				self.map_viewer.aruco_markers[aruco_id] = aruco_marker
				self.map_viewer.last_moved_marker_positions[aruco_id] = position

	def clear_found_markers(self):
		try:
			self.roslink.clear_found_markers()
			self.found_marker_table.clear()
			for aruco_marker in self.map_viewer.aruco_markers.values():
				aruco_marker.setIcon(self.map_viewer.old_aruco_icon)
		except rospy.service.ServiceException as e:
			QMessageBox(
				QMessageBox.Critical,
				"Failed to clear found markers.",
				f"Could not clear found markers.",
				QMessageBox.Ok,
			).exec()
			rospy.logerr(e)

### route optimizer ##########################################################

	def optimize_route_handler(self):
		def failed_to_reorder_markers():
			# TODO: popup box
			rospy.logwarn('Failed to reorder markers!')

		dialog_box = OptimizationDialogBox(algorithms.keys(), cost_functions.keys())

		# handle user request
		if dialog_box.exec():

			# read selected values from dialog box
			algorithm = dialog_box.algorithm_combo.currentText()
			cost_function = dialog_box.cost_func_combo.currentText()
			optimization_start = dialog_box.optimization_start
			ignore_intermediary_markers = dialog_box.ignore_intermediary_markers_checkbox.isChecked()

			# optimize route
			pre_optimized_marker_list = self.marker_list
			robot_latlong = self.map_viewer.last_moved_robot_position
			robot_gps = GeoPoint(robot_latlong[0], robot_latlong[1], 0)
			optimized_marker_list, route_cost = optimize_route(
				robot_gps, self.marker_list, ignore_intermediary_markers, algorithm, cost_function, optimization_start
			)

			# preview route
			self.disp_aut_markers(optimized_marker_list)

			# ask user to confirm optimization
			confirmation_box = QMessageBox(
				QMessageBox.Question,
				f"Confirm Optimized Route?",
				f"Are you sure you want to use the optimized route?\nTotal Cost: {route_cost:.2f}",
				QMessageBox.Yes | QMessageBox.No,
			)

			# send request to reorder markers, with confirmation
			if confirmation_box.exec() == QMessageBox.Yes:
				marker_ids = [marker.marker_id for marker in optimized_marker_list.markers]
				try:
					self.autosave_enabled = True
					self.roslink.reorder_markers(marker_ids)
				except rospy.service.ServiceException as e:
					failed_to_reorder_markers()
					rospy.logerr(e)
			else:
				# display previous markers again
				self.disp_aut_markers(pre_optimized_marker_list)

class OptimizationDialogBox(QDialog):
	def __init__(self, algorithm_items, cost_func_items):
		super().__init__()

		ui = Ui_optimization_dialog()
		ui.setupUi(self)

		self.algorithm_combo = ui.algorithm_combo
		self.cost_func_combo = ui.cost_func_combo

		self.algorithm_combo.addItems(algorithm_items)
		self.cost_func_combo.addItems(cost_func_items)

		self.ignore_intermediary_markers_checkbox = ui.ignore_intermediary_markers_checkbox

		self.marker1_radio = ui.marker1_radio
		self.marker1_radio.toggled.connect(self.marker1_toggled)

		self.marker2_radio = ui.marker2_radio
		self.marker2_radio.toggled.connect(self.marker2_toggled)

		self.optimization_start = 0

	def marker1_toggled(self):
		if self.marker1_radio.isChecked():
			self.optimization_start = 0

	def marker2_toggled(self):
		if self.marker2_radio.isChecked():
			self.optimization_start = 1

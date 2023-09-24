#!/usr/bin/env python3.8

from typing import List

import rospy

from rosgraph_msgs.msg import Log
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import Image

from aruco_finder.msg import FoundMarkerList
from marker_interfacing.srv import AddMarker, AddMarkerRequest, AddMarkerResponse,\
	ClearMarkers, ClearMarkersRequest, ClearMarkersResponse,\
	ReorderMarker, ReorderMarkerRequest, ReorderMarkerResponse,\
	ReorderMarkers, ReorderMarkersRequest, ReorderMarkersResponse,\
	EditMarker, EditMarkerRequest, EditMarkerResponse,\
	RemoveMarker, RemoveMarkerRequest, RemoveMarkerResponse,\
	InsertMarker, InsertMarkerRequest, InsertMarkerResponse
from marker_interfacing.msg import GeodeticMarkerList

from PyQt5.QtCore import QObject, pyqtSignal as Signal

from urc_gui_common.camera_link import CameraFunnel
from urc_gui_common.widgets import MapPoint

class RosLink(QObject):
	"""Supply signals for the Qt app, originating from topic callbacks."""

	pose = Signal(Pose)
	gps = Signal(GeoPoint)
	marker_list = Signal(GeodeticMarkerList)
	state = Signal(String)
	planner_status = Signal(String)
	found_marker_list = Signal(FoundMarkerList)
	current_goal = Signal(MapPoint)
	rosout = Signal(Log)

	# this is where all cameras will send their images and then it gets sent back out appropriately
	camera_funnel_signal = Signal(str, Image)

	def __init__(self):
		"""Initialize the node and all subscriptions."""
		super().__init__()

		rospy.init_node("gui_ros_link", anonymous=True)

		# read ros related launch params
		local_position_topic = rospy.get_param("~local_position_topic")
		global_position_topic = rospy.get_param("~global_position_topic")
		global_origin_topic = rospy.get_param("~global_origin_topic")
		marker_list_topic = rospy.get_param("~marker_list_topic")
		state_topic = rospy.get_param("~state_topic")
		planner_status_topic = rospy.get_param("~planner_status_topic")
		found_marker_list_topic = rospy.get_param("~found_marker_list_topic")
		add_marker_service = rospy.get_param("~add_marker_service")
		clear_markers_service = rospy.get_param("~clear_markers_service")
		reorder_marker_service = rospy.get_param("~reorder_marker_service")
		reorder_markers_service = rospy.get_param("~reorder_markers_service")
		edit_marker_service = rospy.get_param("~edit_marker_service")
		remove_marker_service = rospy.get_param("~remove_marker_service")
		insert_marker_service = rospy.get_param("~insert_marker_service")
		clear_found_markers_service = rospy.get_param("~clear_found_markers_service")

		# create publishers and subscribers
		self.pose_sub = self.make_subscriber(local_position_topic, Pose, self.pose)
		self.gps_sub = self.make_subscriber(global_position_topic, GeoPoint, self.gps)
		self.global_origin_sub = rospy.Subscriber(global_origin_topic, GeoPoint, self.global_origin_callback)
		self.marker_list_sub = self.make_subscriber(marker_list_topic, GeodeticMarkerList, self.marker_list)
		self.state_sub = self.make_subscriber(state_topic, String, self.state)
		self.planner_status_sub = self.make_subscriber(planner_status_topic, String, self.planner_status)
		self.found_marker_sub = self.make_subscriber(found_marker_list_topic, FoundMarkerList, self.found_marker_list)
		self.add_marker_srv = rospy.ServiceProxy(add_marker_service, AddMarker)
		self.clear_markers_srv = rospy.ServiceProxy(clear_markers_service, ClearMarkers)
		self.reorder_marker_srv = rospy.ServiceProxy(reorder_marker_service, ReorderMarker)
		self.reorder_markers_srv = rospy.ServiceProxy(reorder_markers_service, ReorderMarkers)
		self.edit_marker_srv = rospy.ServiceProxy(edit_marker_service, EditMarker)
		self.remove_marker_srv = rospy.ServiceProxy(remove_marker_service, RemoveMarker)
		self.insert_marker_srv = rospy.ServiceProxy(insert_marker_service, InsertMarker)
		self.clear_found_markers_srv = rospy.ServiceProxy(clear_found_markers_service, Trigger)

		# camera funnel
		self.camera_funnel = CameraFunnel(self.camera_funnel_signal)

		# rosout log messages
		self.rosout_sub = self.make_subscriber("rosout_agg", Log, self.rosout)

		# global origin
		self.global_origin = GeoPoint()

	def add_marker(self, lat: float, lon: float, alt: float, error: float, marker_type: str, aruco_id: int, aruco_id_2: int) -> AddMarkerResponse:
		return self.add_marker_srv(AddMarkerRequest(lat, lon, alt, error, marker_type, aruco_id, aruco_id_2))

	def reorder_marker(self, marker_id: int, new_following_marker_id: int) -> ReorderMarkerResponse:
		return self.reorder_marker_srv(ReorderMarkerRequest(marker_id, new_following_marker_id))
	
	def reorder_markers(self, marker_ids: List[int]) -> ReorderMarkersResponse:
		return self.reorder_markers_srv(ReorderMarkersRequest(marker_ids))

	def edit_marker(self, lat: float, lon: float, alt: float, error: float, marker_type: str, aruco_id: int, aruco_id_2: int, marker_id: int) -> EditMarkerResponse:
		return self.edit_marker_srv(EditMarkerRequest(lat, lon, alt, error, marker_type, aruco_id, aruco_id_2, marker_id))

	def remove_marker(self, marker_id) -> RemoveMarkerResponse:
		return self.remove_marker_srv(RemoveMarkerRequest(marker_id))
	
	def insert_marker(self, lat: float, lon: float, alt: float, error: float, marker_type: str, aruco_id: int, aruco_id_2: int, new_following_marker_id: int) -> InsertMarkerResponse:
		return self.insert_marker_srv(InsertMarkerRequest(lat, lon, alt, error, marker_type, aruco_id, aruco_id_2, new_following_marker_id))

	def clear_markers(self) -> ClearMarkersResponse:
		return self.clear_markers_srv(ClearMarkersRequest())

	def clear_found_markers(self) -> TriggerResponse:
		return self.clear_found_markers_srv(TriggerRequest())

	def make_subscriber(self, topic_name: str, topic_type: type, signal: Signal):
		return rospy.Subscriber(
			topic_name,
			topic_type,
			lambda data: signal.emit(data),
		)
	
	def global_origin_callback(self, global_origin: GeoPoint):
		self.global_origin = global_origin

if __name__ == "__main__":
	rospy.loginfo("Starting GUI ROS link")
	roslink = RosLink()
	rospy.spin()

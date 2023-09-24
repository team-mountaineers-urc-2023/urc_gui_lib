import math
from copy import deepcopy
from itertools import permutations
from typing import List

import pymap3d as pm

import rospy

from geographic_msgs.msg import GeoPoint

from marker_interfacing.msg import GeodeticMarkerList, GeodeticMarker

### helpers ##################################################################

def geo_dist(pt1: GeoPoint, pt2: GeoPoint):
	x, y, z = pm.geodetic2enu(
		pt1.latitude, pt1.longitude, pt1.altitude,
		pt2.latitude, pt2.longitude, pt2.altitude,
	)
	return math.sqrt(x**2 + y**2)

def route_cost(initial_position: GeoPoint, route: List[GeodeticMarker], cost_func):
	if len(route) < 1:
		return 0

	cost = cost_func(initial_position, route[0], 0)
	route_len = len(route)
	for i in range(1, route_len):
		cost += cost_func(route[i - 1].gps, route[i], i / route_len)

	return cost

arrival_success_rate = {
	'no_marker': .9,
	'one_marker': .75,
	'gate': .5,
	'intermediary_marker': 1
}

### cost functions ###########################################################

def _distance(initial_position: GeoPoint, target_marker: GeodeticMarker, route_completion: float) -> float:
	return geo_dist(initial_position, target_marker.gps)

def _arrival_complexity(initial_position: GeoPoint, target_marker: GeodeticMarker, route_completion: float) -> float:
	return (10000 / arrival_success_rate[target_marker.marker_type]) + geo_dist(initial_position, target_marker.gps)

def _search_radius(initial_position: GeoPoint, target_marker: GeodeticMarker, route_completion: float) -> float:
	return 10000 * target_marker.waypoint_error + geo_dist(initial_position, target_marker.gps)

def _shubhs_rover_cost_function(initial_position: GeoPoint, target_marker: GeodeticMarker, route_completion: float) -> float:
	dist = geo_dist(initial_position, target_marker.gps)
	search_radius = target_marker.waypoint_error

	expected_attempts = 1 / arrival_success_rate[target_marker.marker_type]
	expected_lengths_traveled = 1 + 2*(expected_attempts - 1)

	travel_cost = dist**1.25 * expected_lengths_traveled
	search_cost = search_radius**2 * (1 - route_completion)**.3

	return travel_cost + search_cost

def _no_cost(initial_position: GeoPoint, target_marker: GeodeticMarker, route_completion: float) -> float:
	return 0

cost_functions = {
	"Distance Only": _distance,
	"Arrival Complexity Only": _arrival_complexity,
	"Search Radius Only": _search_radius,
	"Shubh's Rover Cost Function": _shubhs_rover_cost_function
}

### algorithms ###############################################################

def _greedy(initial_position: GeoPoint, unoptimized_markers: List[GeodeticMarker], cost_func) -> List[GeodeticMarker]:

	optimized_markers = []

	# determine new marker order
	curr_position = initial_position
	marker_count = len(unoptimized_markers)
	while unoptimized_markers:

		# find lowest cost marker
		min_cost = float('inf')
		for index, marker in enumerate(unoptimized_markers):
			cost_to_marker = cost_func(curr_position, marker, index / marker_count)
			if cost_to_marker < min_cost:
				lowest_cost_marker_index = index
				min_cost = cost_to_marker

		# move lowest cost marker from unoptimized marker list to optimized marker list
		lowest_cost_marker = unoptimized_markers.pop(lowest_cost_marker_index)
		optimized_markers.append(lowest_cost_marker)

		# lowest cost marker becomes new current position for next iteration
		curr_position = lowest_cost_marker.gps

	return optimized_markers

def _brute_force(initial_position: GeoPoint, unoptimized_markers: List[GeodeticMarker], cost_func) -> List[GeodeticMarker]:

	# generate all possible routes
	all_routes = permutations(unoptimized_markers, len(unoptimized_markers))

	# find route with the minimum cost
	min_cost = float('inf')
	for route in all_routes:
		cost = route_cost(initial_position, route, cost_func)
		if cost < min_cost:
			min_cost = cost
			min_cost_route = route

	return list(min_cost_route)

def _no_algorithm(initial_position: GeoPoint, unoptimized_markers: List[GeodeticMarker], cost_func) -> List[GeodeticMarker]:
	return unoptimized_markers

algorithms = {
	"Brute Force": _brute_force,
	"Greedy": _greedy,
	"None": _no_algorithm,
}

### main function ############################################################

def optimize_route(initial_position: GeoPoint, marker_list: GeodeticMarkerList, ignore_intermediary=False, algorithm_name='Greedy', cost_func_name='Lowest Distance', optimization_start=0, optimization_end=None):

	# remove intermediary markers, if neeeded
	if ignore_intermediary:
		unoptimized_markers = [marker for marker in marker_list.markers if marker.marker_type != 'intermediary_marker']
	else:
		unoptimized_markers = deepcopy(marker_list.markers)

	# defaults
	optimization_end = optimization_end or len(unoptimized_markers)

	# input validation
	try:
		algorithm = algorithms[algorithm_name]
	except KeyError:
		rospy.logerr('Invalid algorithm name for route optimization. Returning unoptimized list.')
		return marker_list, route_cost(initial_position, unoptimized_markers, cost_func)

	try:
		cost_func = cost_functions[cost_func_name]
	except KeyError:
		rospy.logerr('Invalid cost function name for route optimization. Returning unoptimized list.')
		return marker_list, route_cost(initial_position, unoptimized_markers, _no_cost)
	
	if len(unoptimized_markers) < 2:
		# nothing to optimize
		return marker_list, route_cost(initial_position, unoptimized_markers, cost_func)

	if not (0 <= optimization_start < len(unoptimized_markers) - 1):
		rospy.logerr('Invalid optimization start for route optimization. Returning unoptimized list.')
		return marker_list, route_cost(initial_position, unoptimized_markers, cost_func)

	if not (1 < optimization_end <= len(unoptimized_markers)):
		rospy.logerr('Invalid optimization end for route optimization. Returning unoptimized list.')
		return marker_list, route_cost(initial_position, unoptimized_markers, cost_func)

	# determine what to optimize (optimization_start-optimization_end)
	pos0 = initial_position if optimization_start == 0 else unoptimized_markers[optimization_start - 1].gps
	beginning_markers_to_ignore = unoptimized_markers[:optimization_start]
	markers_to_optimize = unoptimized_markers[optimization_start:optimization_end]
	end_markers_to_ignore = unoptimized_markers[optimization_end:]

	# optimize markers
	optimized_markers = algorithm(pos0, markers_to_optimize, cost_func)

	# compute route cost
	cost_of_route = route_cost(pos0, optimized_markers, cost_func)

	# create new marker list
	optimized_marker_list = GeodeticMarkerList()
	optimized_marker_list.markers = beginning_markers_to_ignore + optimized_markers + end_markers_to_ignore
	optimized_marker_list.count = len(optimized_markers)

	return optimized_marker_list, cost_of_route

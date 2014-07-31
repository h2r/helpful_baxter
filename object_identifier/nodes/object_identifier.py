#!/usr/bin/env python

import roslib
roslib.load_manifest("object_identifier")
import rospy

from object_recognition_msgs.msg import RecognizedObjectArray, RecognizedObject
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import PoseWithCovarianceStamped

defined_objects = [
	{"class": "gyrobowl", "name": "cocoa_bowl"},
	{"class": "gyrobowl", "name": "flour_bowl"},
	{"class": "gyrobowl", "name": "salt_bowl"},
	{"class": "gyrobowl", "name": "baking_powder_bovl"},
	{"class": "gyrobowl", "name": "baking_soda_bowl"},
	{"class": "gyrobowl", "name": "eggs_bowl"},
	{"class": "gyrobowl", "name": "sugar_bowl"},
	{"class": "gyrobowl", "name": "vanilla_bowl"},
	{"class": "gyrobowl", "name": "butter_bowl"},
	{"class": "mixing_bowl", "name": "mixing_bowl"},
	{"class": "spoon", "name": "wooden_spoon"},
	{"class": "spoon", "name": "wooden_spoon"},
]


class ObjectIdentifier:
	def __init__(self):
		#self.ork_objects = []
		#self.joberlin_objects = []
		#rospy.Subscriber("/recognized_object_array", self.ork_callback)
		#rospy.Subscriber("/joberlin_detection", self.joberlin_callback)
		rospy.Subscriber("/ar_markers", self.ar_marker_callback)
		self.ar_object_publisher = rospy.Publisher("/ar_objects", RecognizedObjectArray)

	def ork_callback(self, msg):
		self.ork_objects = msg.objects

	def joberlin_callback(self, msg):
		self.joberlin_objects = msg.objects

	def ar_marker_callback(self, msg):
		objects_msg = RecognizedObjectArray()
		objects_msg.header = msg.header
		for marker in msg.markers:
			object = RecognizedObject()
			object.pose = self.get_pose_from_pose(marker.pose)
			object.type.key = defined_objects[marker.id]["name"]
			objects_msg.objects.append(object)
		self.ar_object_publisher.publish(objects_msg)

	def get_pose_from_pose(self, pose_stamped):
		pose = PoseWithCovarianceStamped()
		pose.header = pose_stamped.header
		pose.pose.pose = pose_stamped.pose
		return pose

if __name__ == "__main__":
	rospy.init_node("object_identifier")
	identifier = ObjectIdentifier()
	rospy.spin()
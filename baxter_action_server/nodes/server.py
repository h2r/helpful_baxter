#!/usr/bin/env python

import roslib
roslib.load_manifest("baxter_action_server")

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
import genpy
import random
import traceback
import math
import actionlib

from move_msgs.msg import BaxterAction
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from moveit_msgs.msg import Grasp, PlanningScene
from object_recognition_msgs.msg import RecognizedObjectArray
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf.transformations import quaternion_from_euler
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
#from meldon_detection.msg import MarkerObjectArray, MarkerObject
from baxter_grasps_server.srv import GraspService, GraspServiceRequest
from ar_track_alvar.msg import AlvarMarker, AlvarMarkers
from threading import Thread
from visualization_msgs.msg import Marker
from baxter_pick_and_place.move_helper import MoveHelper
from baxter_grasps_server.grasping_helper import GraspingHelper


region_lookup = {
	"counter": (0.6, -0.25, 0.3),
	"sink": (0.7, 0.4, 0.5),
	"robot_counter": (0.6, 0.0, 0.0)
}

class ActionServer:
	def __init__(self):
		self.transformer = TransformListener()
		self.object_poses = dict()
		self.is_picking = False
		self.is_placing = False
		self.markers_publisher = rospy.Publisher("/grasp_markers", Marker)
		
		self.graspService = rospy.ServiceProxy('grasp_service', GraspService)
		self.group = moveit_commander.MoveGroupCommander("left_arm")
		self.group.set_workspace([0.0, -0.2, -0.30, 0.9, 1.0, 2.0] )
		rospy.Subscriber("baxter_action", BaxterAction, self.actionCB)
		rospy.Subscriber("/ar_objects", RecognizedObjectArray, self.markers_callback)
		

	def is_picking_or_placing(self):
		return self.is_picking or self.is_placing


	def pick_and_place(self, object_id, object_pose, destination_point):
		object_name = object_id

		print("Attempting to pick up object " + object_name)
		
		self.is_picking = True
		
		rospy.loginfo("Finding a valid place pose")
		place_poses = self.getValidPlacePoses(destination_point)

		rospy.loginfo("Attempting to pick up object " + object_name)
		MoveHelper.move_to_neutral("left", True)

		pickSuccess = False
		try:
			pickSuccess = self.pick(object_pose, object_name)
			rospy.loginfo("Pick success: " + str(pickSuccess))
		except Exception as e:
			traceback.print_exc()
			raise e
		finally:
			self.is_placing = pickSuccess
			self.is_picking = False

		if not pickSuccess:
			rospy.logerr("Object pick up failed")
			return
		
		place_result = False
		try:
			for place_pose in place_poses:
				rospy.loginfo("Attempting to place object")
				if self.place(object_name, object_pose, place_pose):
					break
		except Exception as e:
			traceback.print_exc()
			raise e
		finally:
			self.is_placing = False

	def pick(self, object_pose, object_name):
		self.group.detach_object()			

		graspResponse = self.graspService(object_name)
		if not graspResponse.success:
			rospy.logerr("No grasps were found for object " + object_name)
			return

		self.group.set_planning_time(20)
		self.group.set_start_state_to_current_state()

		grasps = MoveHelper.set_grasps_at_pose(object_pose, graspResponse.grasps, self.transformer)
		self.publishMarkers(grasps, object_name)
		
		result = self.group.pick(object_name, grasps * 5)
		return result

	def place(self, object_id, original_pose, place_pose):
		goal_pose = copy.deepcopy(original_pose)
		goal_pose.pose.position.x = place_pose.pose.position.x
		goal_pose.pose.position.y = place_pose.pose.position.y
		result = self.group.place(object_id, goal_pose)
		return result

	def getValidPlacePoses(self, place_point):
		
		place_poses = []
		for i in range(36):	
			place_pose = PoseStamped()
			place_pose.header.frame_id = "world"
			place_pose.pose.position.x = place_point[0] #convert back to meters
			place_pose.pose.position.y = place_point[1]
			place_pose.pose.position.z = place_point[2]
			quat = quaternion_from_euler(0, math.pi/2.0, i * 2.0 * math.pi / 36.0)
			place_pose.pose.orientation.x = quat[0]
			place_pose.pose.orientation.y = quat[1]
			place_pose.pose.orientation.z = quat[2]
			place_pose.pose.orientation.w = quat[3]
			place_poses.append(place_pose)
		return place_poses

	def actionCB(self, msg):

		if msg.type == BaxterAction.MOVE:
			if not self.is_picking_or_placing():
				object_name = msg.move_action.object.name

				if object_name in self.object_poses.keys():
					destination_point = region_lookup[msg.move_action.region.name]
					poses = copy.deepcopy(self.object_poses)
					self.current_thread = Thread(None, self.pick_and_place, None, (object_name, poses[object_name], destination_point))
					self.current_thread.start()
				else:
					self.current_thread = Thread(None, MoveHelper.move_to_neutral, None, ("left", True))
					self.current_thread.start()

	def markers_callback(self, msg):
		self.object_poses = dict()
		
		for object in msg.objects:
			pose = GraspingHelper.getPoseStampedFromPoseWithCovariance(object.pose)
			self.object_poses[str(object.type.key)] = pose
	
	def publishMarkers(self, grasps, object_name):
		markers = MoveHelper.create_grasp_markers(grasps, object_name)
		for marker in markers:
			self.markers_publisher.publish(marker)

	def go(self):
		rospy.spin()

if __name__ == "__main__":
	rospy.init_node("baxter_action_server")
	server = ActionServer();
	server.go();
#!/usr/bin/env python

import roslib
roslib.load_manifest("baxter_action_server")

import rospy

class ActionServer:

	def go(self):
		rospy.spin()

if __name__ == "__main__":

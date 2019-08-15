#!/usr/bin/env python
import rospy
import numpy as np
import tf
from octomap_msgs.msg import Octomap
from moveit_msgs.msg import PlanningScene
from std_msgs.msg import Bool


class OctoHandler():
	mapMsg = None

	def __init__(self):
		rospy.init_node('moveit_octomap_handler')
		rospy.Subscriber('/octomap_full_static', Octomap, self.sub_octomap, queue_size=10)
		rospy.Subscriber('/planning_scene/update', Bool, self.sub_sw, queue_size=1)
		pub_planning_scene = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)
		r = rospy.Rate(10)

		# Setup about planning scene
		self.ps = PlanningScene()
		self.ps.world.octomap.header.frame_id = '/map'
		self.ps.is_diff = True

		self.sw = True

		while not rospy.is_shutdown():
			if(self.mapMsg is not None):
				pub_planning_scene.publish(self.mapMsg)
				# print("Publish OctomapMessage to PlanningScene\n")
			else:
				print("pass")
				pass
			r.sleep()

	def sub_sw(self, msg):
		self.sw = msg.data

	def sub_octomap(self, msg):
		self.ps.world.octomap.header.stamp = rospy.Time.now()
		if self.sw == True:
			self.ps.world.octomap.octomap = msg
			self.mapMsg = self.ps
		else:
			print "Stop to update the Planning Scene."


if __name__ == '__main__':
	Octo = OctoHandler()
	rospy.spin()

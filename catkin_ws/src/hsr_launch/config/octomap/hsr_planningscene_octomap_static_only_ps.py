#!/usr/bin/env python
import rospy
from moveit_msgs.msg import PlanningScene
from std_msgs.msg import Bool
import numpy as np


class OctoHandler():

	def __init__(self):
		rospy.init_node('moveit_octomap_only_ps_handler')
		rospy.Subscriber('/planning_scene/update', Bool, self.sub_sw, queue_size=1)
		pub_planning_scene = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)

		r = rospy.Rate(10)

		octo_data = np.loadtxt("/root/HSR/catkin_ws/src/hsr_launch/config/octomap/octo_data.csv")

		# Setup about planning scene
		ps = PlanningScene()
		ps.world.octomap.header.frame_id = '/map'
		ps.world.octomap.octomap.header.frame_id = '/map'
		ps.world.octomap.octomap.binary = False
		ps.world.octomap.octomap.id = 'OcTree'
		ps.world.octomap.octomap.resolution = 0.04
		ps.is_diff = True

		self.planning_scene_update_sw = True

		while not rospy.is_shutdown():
			ps.world.octomap.header.stamp = rospy.Time.now()
			ps.world.octomap.octomap.header.stamp = rospy.Time.now()

			if self.planning_scene_update_sw == True:

				ps.world.octomap.octomap.data = octo_data
				pub_planning_scene.publish(ps)
				rospy.loginfo("Publish OctomapMessage to PlanningScene.")

			else:
				rospy.loginfo("Stop to update the Planning Scene.")
			r.sleep()

	def sub_sw(self, msg):
		self.planning_scene_update_sw = msg.data


if __name__ == '__main__':
	Octo = OctoHandler()
	rospy.spin()

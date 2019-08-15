#!/usr/bin/env python
import rospy
from octomap_msgs.srv import GetOctomap, GetOctomapRequest
from moveit_msgs.msg import PlanningScene
from std_msgs.msg import Bool
import numpy as np


class OctoHandler():

	def __init__(self):
		rospy.init_node('get_octomap')
		rospy.wait_for_service('/octomap_full_static')
		srv_octo_static = rospy.ServiceProxy('/octomap_full_static', GetOctomap)

		r = rospy.Rate(10)
		req_octomap = GetOctomapRequest()

		octo_data = None
		sw = True

		try:
			octo_data = srv_octo_static(req_octomap)
		except rospy.ServiceException, e:
			rospy.loginfo("[Service /octomap_server/get_octomap/client] call failed: %s", e)
		if octo_data != None and octo_data.map.data != []:
			if sw:
				np.savetxt("octo_data.csv", octo_data.map.data)
			print "Got octomap data."
		else:
			print "Cannot get octomap data."

if __name__ == '__main__':
	Octo = OctoHandler()
	rospy.spin()

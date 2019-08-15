#!/usr/bin/env python
import rospy
from octomap_msgs.srv import GetOctomap, GetOctomapRequest
from moveit_msgs.msg import PlanningScene
from std_msgs.msg import Bool


class OctoHandler():

	def __init__(self):
		rospy.init_node('moveit_octomap_handler')
		rospy.Subscriber('/planning_scene/update', Bool, self.sub_sw, queue_size=1)
		pub_planning_scene = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)
		rospy.wait_for_service('/octomap_full_dynamic')
		srv_octo_dynamic = rospy.ServiceProxy('/octomap_full_dynamic', GetOctomap)

		r = rospy.Rate(5)
		req_octomap = GetOctomapRequest()

		# Setup about planning scene
		ps = PlanningScene()
		ps.world.octomap.header.frame_id = '/map'
		ps.is_diff = True

		octo_data = None
		self.planning_scene_update_sw = True

		while not rospy.is_shutdown():
			ps.world.octomap.header.stamp = rospy.Time.now()

			if self.planning_scene_update_sw == True:
				try:
					octo_data = srv_octo_dynamic(req_octomap)
				except rospy.ServiceException, e:
					rospy.loginfo("[Service /octomap_server/get_octomap/client] call failed: %s", e)
				if octo_data != None and octo_data.map.data != []:
					ps.world.octomap.octomap = octo_data.map
					pub_planning_scene.publish(ps)
					rospy.loginfo("Publish OctomapMessage to PlanningScene.")
				else:
					pub_planning_scene.publish(ps)
					rospy.loginfo("Unable to get the Octomap data.")
			else:
				rospy.loginfo("Stop to update the Planning Scene.")
			r.sleep()

	def sub_sw(self, msg):
		self.planning_scene_update_sw = msg.data


if __name__ == '__main__':
	Octo = OctoHandler()
	rospy.spin()

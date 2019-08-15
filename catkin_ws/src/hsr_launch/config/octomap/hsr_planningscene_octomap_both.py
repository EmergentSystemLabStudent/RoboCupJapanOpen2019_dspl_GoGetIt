#!/usr/bin/env python
import rospy
from octomap_msgs.srv import GetOctomap, GetOctomapRequest
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld
from std_msgs.msg import Bool


class OctoHandler():

	def __init__(self):
		rospy.init_node('moveit_octomap_handler')
		rospy.Subscriber('/planning_scene/update', Bool, self.sub_sw, queue_size=1)
		pub_planning_scene = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)
		pub_planning_scene_world = rospy.Publisher('/planning_scene_world', PlanningSceneWorld, queue_size=10)
		rospy.wait_for_service('/octomap_full_static')
		rospy.wait_for_service('/octomap_full_dynamic')
		srv_octo_static = rospy.ServiceProxy('/octomap_full_static', GetOctomap)
		srv_octo_dynamic = rospy.ServiceProxy('/octomap_full_dynamic', GetOctomap)

		r = rospy.Rate(10)
		req_octomap = GetOctomapRequest()

		# Setup about planning scene by dynamic octomap
		ps = PlanningScene()
		ps.world.octomap.header.frame_id = '/map'
		ps.is_diff = True

		# Setup about planning scene by static octomap
		psw = PlanningSceneWorld()
		psw.octomap.header.frame_id = '/map'


		self.planning_scene_update_sw = True

		while not rospy.is_shutdown():
			ps.world.octomap.header.stamp = rospy.Time.now()
			psw.octomap.header.stamp = rospy.Time.now()

			if self.planning_scene_update_sw == True:
				try:
					octo_data_s = srv_octo_static(req_octomap)
					octo_data_d = srv_octo_dynamic(req_octomap)
				except rospy.ServiceException, e:
					rospy.loginfo("[Service /octomap_server/get_octomap/client] call failed: %s", e)
				if octo_data_s.map.data != [] and octo_data_d.map.data != []:
					ps.world.octomap.octomap = octo_data_d.map
					psw.octomap.octomap.data = octo_data_s.map.data
					pub_planning_scene_world.publish(psw)
					pub_planning_scene.publish(ps)
					rospy.loginfo("Publish OctomapMessage to PlanningScene.")
				else:
					rospy.loginfo("Unable to get the Octomap data.")
			else:
				rospy.loginfo("Stop to update the Planning Scene.")
			r.sleep()

	def sub_sw(self, msg):
		self.planning_scene_update_sw = msg.data


if __name__ == '__main__':
	Octo = OctoHandler()
	rospy.spin()

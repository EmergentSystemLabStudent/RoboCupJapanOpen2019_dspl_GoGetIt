#! /usr/bin/env python

import rospy
import tf
from em_map_record.srv import *
from geometry_msgs.msg import Pose, Quaternion
import hsrb_interface
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')

class GetPose():
    #save position
    def pose_server(self, req):
        place_name = req.name
        fp = open(self.DATA_FOLDER + "/pose.csv",'a')
        p = omni_base.pose
        q = tf.transformations.quaternion_from_euler(0, 0, p[2])
        pose = Pose()
        pose.position.x = p[0]
        pose.position.y = p[1]
        pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
        fp.write(place_name + "," + str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.orientation.z) + "," + str(pose.orientation.w) + "\n")
        fp.close()
        rospy.loginfo("[Service em_map_record/pose] get new pose '%s': (%f,%f,%f,%f)", place_name, pose.position.x, pose.position.y, pose.orientation.z, pose.orientation.w)
        return map_poseResponse(True)

    def __init__(self):
        TASKNAME = rospy.get_param('~task_name')
        self.DATA_FOLDER = "../data/" + TASKNAME
        s = rospy.Service('em_map_record/pose', map_pose, self.pose_server)
        rospy.loginfo("[Service em_map_record/pose] Ready em_map_record/pose")

if __name__ == "__main__":
    hoge = GetPose()
    rospy.spin()

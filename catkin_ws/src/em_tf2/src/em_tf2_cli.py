#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from em_tf2.srv import *

def tf2_client(before, after, before_pose):
    rospy.wait_for_service('em_tf2')
    try:
        em_tf2_srv = rospy.ServiceProxy('em_tf2', em_tf2)
        res = em_tf2_srv(before, after, before_pose)
    except rospy.ServiceException, e:
        rospy.loginfo("[Service em_tf2] call failed: %s", e)
    return res.after_pose

if __name__ == "__main__":
    rospy.init_node('em_tf2_cli')
    before = "base_link"
    after = "head_pan_link"
    before_pose = Pose()
    before_pose.position.x = 1.0
    before_pose.position.y = 2.0
    before_pose.position.z = 3.0

    pose = tf2_client(before, after, before_pose)
    rospy.loginfo("[Service em_tf2] Successful")
    print pose

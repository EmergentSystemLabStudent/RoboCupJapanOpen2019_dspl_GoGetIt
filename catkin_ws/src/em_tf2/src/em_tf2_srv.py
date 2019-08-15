#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped
from em_tf2.srv import *

class tf2(object):
    #before座標系からafter座標系へ座標変換
    def tf2_server(self, req):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        function = tf_buffer.lookup_transform(req.after, req.before, rospy.Time(0), rospy.Duration(1.0))
        before_pose_stamped = PoseStamped()
        before_pose_stamped.pose = req.before_pose
        after_pose_stamped = tf2_geometry_msgs.do_transform_pose(before_pose_stamped, function)
        return em_tf2Response(after_pose_stamped)

    def __init__(self):
        s = rospy.Service('em_tf2', em_tf2, self.tf2_server)
        rospy.loginfo("[Service em_tf2] Ready em_tf2")

if __name__ == '__main__':
    rospy.init_node('em_tf2_srv')
    hoge = tf2()
    rospy.spin()

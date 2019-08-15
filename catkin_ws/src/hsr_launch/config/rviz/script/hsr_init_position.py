#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
#from time import sleep
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitPosition():
    def __init__(self):
        rospy.init_node('init_position_rviz')
        self.sub_cur_pose = rospy.Subscriber('/laser_2d_pose', PoseWithCovarianceStamped, self.sub_currect_pose, queue_size=10)
        self.pub_init_pose = rospy.Publisher('/laser_2d_correct_pose', PoseWithCovarianceStamped, queue_size=10)

        # r = rospy.Rate(0.01)
        # r.sleep()

    def sub_currect_pose(self, msg):
        self.init_pose = msg

        if msg.header.seq < 3:
            print "Initialize the HSR position (Assume only the first time)"
            time_now = rospy.get_rostime()
            self.init_pose.header.seq = 0
            self.init_pose.header.stamp.secs = time_now.secs
            self.init_pose.header.stamp.nsecs = time_now.nsecs
            self.init_pose.pose.pose.position.x = 0.0
            self.init_pose.pose.pose.position.y = 0.0
            self.init_pose.pose.pose.position.z = 0.0
            self.init_pose.pose.pose.orientation.x = 0.0
            self.init_pose.pose.pose.orientation.y = 0.0
            self.init_pose.pose.pose.orientation.z = 0.0
            self.init_pose.pose.pose.orientation.w = 1.0
            self.init_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

            rospy.sleep(0.1)
            self.pub_init_pose.publish(self.init_pose)
            rospy.sleep(0.1)
            self.pub_init_pose.publish(self.init_pose)

if __name__ == '__main__':
    #sleep(20) # Wait for starting Rviz GUI
    ip = InitPosition()
    rospy.spin()

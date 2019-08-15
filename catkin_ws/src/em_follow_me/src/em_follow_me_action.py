#! /usr/bin/env python

import rospy, tf, tf2_ros
import actionlib
import numpy as np
import math

from geometry_msgs.msg import Point,Quaternion,Twist,PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
# from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

from em_follow_me.msg import *
from em_led_control.msg import *
# import hsrb_interface

class Followme_Action:

    def __init__(self):

        self.server = actionlib.SimpleActionServer('em_follow_me_action', follow_meAction, False)
        self.server.register_goal_callback(self.goalCallback)
        self.server.start()
        #self.led_client = actionlib.SimpleActionClient('em_led_action', ledAction)
        rospy.loginfo("start server")

        rospy.Subscriber("/hsrb/base_scan", LaserScan, self.laserCallback, queue_size=1)
        rospy.Subscriber("katsu_point", PointStamped, self.targetCallback, queue_size=1)
        self.pub = rospy.Publisher('init_particle', Bool, queue_size=1)
        self.vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)

        self.target_tf = PointStamped()

        self.pre_vel = Twist()
        self.loop_continue = False

        self.STOP_DISTANCE = rospy.get_param('~stop_distance')
        self.BACK_DISTANCE = rospy.get_param('~back_distance')
        self.MAX_SPEED = rospy.get_param('~max_speed')

        #self.led_client.wait_for_server()

    def targetCallback(self, data):
        self.target_tf = data

    def laserCallback(self, data):

        if not self.loop_continue: return

        self.laser_info = np.c_[np.arange(data.angle_min,data.angle_max,data.angle_increment),data.ranges]

        #try:
        #    tf_buffer = tf2_ros.Buffer()
        #    tf_listener = tf2_ros.TransformListener(tf_buffer)
        #    target_tf = tf_buffer.lookup_transform("base_footprint","trace_target",rospy.Time(0),rospy.Duration(1.0))
        #except Exception as e:
        #    rospy.loginfo("%s", e)
        #    return

        #self.vel_pub.publish(self.tracePosition(target_tf.transform.translation))
        self.vel_pub.publish(self.tracePosition(self.target_tf.point))

    def goalCallback(self):

        self.loop_continue = self.server.accept_new_goal().start
        rospy.loginfo("follow_me server get " + str(self.loop_continue))
        self.pub.publish(True);

        goal = ledGoal()
        goal.mode = 3
        # goal.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.0)
        # goal.flash_interval = 10
        # self.led_client.send_goal(goal, feedback_cb=print_feedback)

        result = follow_meResult()

        rospy.sleep(1.0)

        result.output = True
        self.server.set_succeeded(result)

    def tracePosition(self,target_pos):

        vel = Twist();

        # rotate to person
        vel.angular.z = math.atan2(target_pos.y,target_pos.x)
        # keep distance to person
        vel.linear.x = (target_pos.x - self.STOP_DISTANCE) * 0.8

        self.pre_vel = vel
        return vel


if __name__ == '__main__':

    rospy.init_node('follow_me_action')

    server = Followme_Action()
    rospy.spin()

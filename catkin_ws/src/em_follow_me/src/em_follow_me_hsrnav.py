#! /usr/bin/env python

import rospy, tf, tf2_ros
import actionlib
import numpy as np
import math

from geometry_msgs.msg import Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from em_follow_me.msg import *
import hsrb_interface

sleep_time = 1.0
stop_distance = 0.8

class Followme_Action:

    def __init__(self):

        self.server = actionlib.SimpleActionServer('em_follow_me_action', follow_meAction, self.execute, False)
        self.server.start()
        print "start server"

        self.movebase = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.movebase.wait_for_server()

        self.robot = hsrb_interface.Robot()
        self.omni_base = self.robot.get('omni_base')

        self.pos_list = np.empty(0)

    def trace_point(self,target_pos,robot_pos):

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"

        try:
            self.pos_list = np.append(self.pos_list,[[target_pos.x, target_pos.y, 0]],axis=0)
            d = np.sqrt((self.pos_list[:,0] - target_pos.x)**2 + (self.pos_list[:,1] - target_pos.y)**2)
            self.pos_list = np.delete(self.pos_list,np.where(d>stop_distance),axis=0)
            if np.sqrt((target_pos.x - robot_pos.x)**2 + (target_pos.y - robot_pos.y)**2) > stop_distance:
                rospy.loginfo("go")
                goal.target_pose.pose.position = Point(self.pos_list[0,0],self.pos_list[0,1],0)
            else:
                rospy.loginfo("stay")
                goal.target_pose.pose.position = robot_pos
        except Exception as e:
            rospy.loginfo("%s",e)
            self.pos_list = np.array([[target_pos.x,target_pos.y,0]])
            goal.target_pose.pose.position = robot_pos
            rospy.loginfo("stay")

        rad = math.atan2(target_pos.x-goal.target_pose.pose.position.x,target_pos.y-goal.target_pose.pose.position.y) - np.radians(90.0)
        print np.degrees(-rad)
        q = tf.transformations.quaternion_from_euler(0, 0, -rad)
        goal.target_pose.pose.orientation = Quaternion(0,0,q[2],q[3])

        return goal

    def execute(self,start):

        result = follow_meResult()
        if not start:
            result.output = False
            self.server.set_succeeded(result)
            return

        while not rospy.is_shutdown():

            try:
                tf_buffer = tf2_ros.Buffer()
                tf_listener = tf2_ros.TransformListener(tf_buffer)
                target_tf = tf_buffer.lookup_transform("map","trace_target",rospy.Time(0),rospy.Duration(1.0))
            except Exception as e:
                rospy.loginfo("%s", e)
                continue

            try:
                tf_buffer = tf2_ros.Buffer()
                tf_listener = tf2_ros.TransformListener(tf_buffer)
                robot_tf = tf_buffer.lookup_transform("map","base_footprint",rospy.Time(0),rospy.Duration(1.0))
            except Exception as e:
                rospy.loginfo("%s", e)
                continue

            self.movebase.send_goal(self.trace_point(target_tf.transform.translation,robot_tf.transform.translation))

            rospy.sleep(sleep_time)

        self.server.set_succeeded(result)


if __name__ == '__main__':

    rospy.init_node('follow_me_action')

    server = Followme_Action()
    rospy.spin()
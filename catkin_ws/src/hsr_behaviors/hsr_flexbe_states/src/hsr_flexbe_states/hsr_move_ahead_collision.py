#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Point,Quaternion,Twist,PointStamped

class hsr_Move_Ahead_Collision(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- distance float distance
    <= continue        continue
    '''

    def __init__(self, speed, ros_rate, distance = 0.0):
        super(hsr_Move_Ahead_Collision, self).__init__(outcomes=['continue'])
        self.vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
        self.linear_distance = distance
        self.max_speed = speed
        self.linear_rate = ros_rate

    def execute(self, userdata):
        rate = self.linear_rate
        r = rospy.Rate(rate)
        vel = Twist();
        linear_speed = self.max_speed
        vel.linear.x = linear_speed
        linear_duration = self.linear_distance / linear_speed
        ticks = int(linear_duration * rate)

        for t in range(ticks):
            self.vel_pub.publish(vel)
            r.sleep()
        self.vel_pub.publish(Twist())

        return 'continue'

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

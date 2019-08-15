#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from em_door_detect.srv import *

class DoorDetect(object):
    def ScanCallback(self, msg):
        self.laser = msg

    #URGの前に障害物があるかどうかを計算
    def DoorService(self, req):
        center = len(self.laser.ranges)/2
        rad = math.atan2(self.door_width_half,self.door_distance)
        range_i = rad / self.laser.angle_increment
        count = 0
        mean = 0
        for i in range(int(-range_i), int(range_i)):
            mean += self.laser.ranges[center+i]
            count += 1
        mean /= count
        #door is open
        if mean > self.door_distance:
            self.open = True
        #door is close
        else:
            self.open = False
        return em_door_detectResponse(self.open)

    def __init__(self):
        self.door_distance = rospy.get_param('~door_distance')
        self.door_width_half = rospy.get_param('~door_width') / 2
        self.laser = LaserScan()
        self.open = False
        rospy.Subscriber("/hsrb/base_scan",LaserScan,self.ScanCallback, queue_size=10)
        s = rospy.Service("em_door_detect", em_door_detect, self.DoorService)
        rospy.loginfo("[Service em_door_detect] Ready em_door_detect")

if __name__ == '__main__':
    rospy.init_node('em_door_detect_srv', anonymous=True)
    msg = DoorDetect()
    rospy.spin()

#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from em_door_detect.srv import *

def door_client(request):
    fin = False
    rospy.wait_for_service("em_door_detect")
    while fin == False:
        try:
            em_door_detect_srv = rospy.ServiceProxy("em_door_detect", em_door_detect)
            res = em_door_detect_srv(request)
            fin = True
        except rospy.ServiceException, e:
            rospy.loginfo("[Service door_detect] call failed: %s", e)
    return res.result

if __name__ == '__main__':
    rospy.init_node('em_door_detect_client', anonymous=True)
    result = door_client(True)
    if result:
        rospy.loginfo("[Service door_detect] open")
    else:
        rospy.loginfo("[Service door_detect] close")

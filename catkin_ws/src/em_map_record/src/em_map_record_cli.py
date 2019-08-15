#!/usr/bin/env python

import rospy
from em_map_record.srv import *
import subprocess

def example_record_client(sentence):
    fin = False
    rospy.wait_for_service('em_map_record/record')
    while fin == False:
        try:
            m_record = rospy.ServiceProxy('em_map_record/record', map_record)
            res = m_record(sentence)
            fin = res.finish
        except rospy.ServiceException, e:
            rospy.loginfo("[Service em_map_record/record] call failed: %s", e)
    return fin

if __name__ == "__main__":
    rospy.init_node('em_map_record_cli')
    TASKNAME = rospy.get_param('~task_name')

    while not rospy.is_shutdown():
        print "'save'     : save map\n'finish'   : finish this node\nthe others : record pose & place name with input key\nplease input the key"
        name = raw_input()
        if name == "save":
            p = subprocess.Popen("rosrun map_server map_saver map:=/update_map -f /root/HSR/catkin_ws/src/em_map_record/data/" + TASKNAME + "/map", shell=True)
            continue
        elif name == "finish":
            break
        else:
            result = example_record_client(name)
            if result:
                rospy.loginfo("[Service em_map_record/record] Successful")

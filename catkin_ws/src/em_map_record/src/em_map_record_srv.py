#!/usr/bin/env python

import rospy
from em_map_record.srv import *
import subprocess

class MapRecord(object):

    def record_server(self, req):
        rospy.wait_for_service('em_map_record/pose')
        rospy.wait_for_service('em_map_record/image')
        try:
            pose_result = self.m_pose(req.name)
            image_result = self.m_image(self.count)
            self.count += 1
        except Exception as e:
            rospy.loginfo("[Service em_map_record/record] %s", e)
            return map_recordResponse(False)
        if not pose_result and not image_result:
            return map_recordResponse(False)
        return map_recordResponse(True)

    def __init__(self):
        new_data = rospy.get_param('~new_data')
        TASKNAME = rospy.get_param('~task_name')
        DATA_FOLDER = "../data/" + TASKNAME
        if new_data:
            p = subprocess.Popen("rm -rf " + DATA_FOLDER, shell=True)
            rospy.sleep(1.0)
            p = subprocess.Popen("mkdir -p " + DATA_FOLDER + "/image/", shell=True)
        self.m_pose = rospy.ServiceProxy('em_map_record/pose', map_pose)
        self.m_image = rospy.ServiceProxy('em_map_record/image', map_image)
        try:
            self.count = sum(1 for i in open(DATA_FOLDER + '/pose.csv', 'r'))
        except:
            self.count = 0
        s = rospy.Service('em_map_record/record', map_record, self.record_server)
        rospy.loginfo("[Service em_map_record/record] Ready em_map_record/record")

if __name__ == "__main__":
    rospy.init_node('em_map_record_srv')
    hoge = MapRecord()
    rospy.spin()

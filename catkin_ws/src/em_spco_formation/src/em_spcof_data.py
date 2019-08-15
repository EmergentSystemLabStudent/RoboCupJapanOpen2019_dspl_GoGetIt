#!/usr/bin/env python

from __init__ import *

import subprocess

class DataServer(object):

    def run_server(self, req):

        try:
            word_count = self.s_word(req.sentence, self.count)
            pose_count = self.s_pose(self.count)
            image_count = self.s_image(self.count)
            self.count = self.count + 1
        except Exception as e:
            rospy.loginfo("[Service spcof/data] %s", e)
            return spcof_dataResponse(False, 0)

        if not word_count and not pose_count and not image_count:
            return spcof_dataResponse(False, 0)

        return spcof_dataResponse(True, self.count)

    def __init__(self):

        new_data = rospy.get_param('~new_data')
        ver_isb = rospy.get_param('~ver_isb')

        self.DATA_FOLDER = DATASET_FOLDER + TRIALNAME
        if new_data:
            p = subprocess.Popen("rm -rf " + self.DATA_FOLDER, shell=True)
            rospy.sleep(1.0)
            p = subprocess.Popen("mkdir -p " + self.DATA_FOLDER + "/image/", shell=True)
            print "true"

        rospy.wait_for_service('em_spco_formation/data/word')
        rospy.wait_for_service('em_spco_formation/data/pose')        
        if ver_isb: rospy.wait_for_service('em_spco_formation/data/yolo')
        else: rospy.wait_for_service('em_spco_formation/data/image')

        self.s_word = rospy.ServiceProxy('em_spco_formation/data/word', spcof_word)
        self.s_pose = rospy.ServiceProxy('em_spco_formation/data/pose', spcof_pose)
        if ver_isb: self.s_image = rospy.ServiceProxy('em_spco_formation/data/yolo', spcof_yolo)
        else: self.s_image = rospy.ServiceProxy('em_spco_formation/data/image', spcof_image)

        try:
            self.count = sum(1 for i in open(self.DATA_FOLDER + '/pose.csv', 'r'))
        except:
            self.count = 0

        s = rospy.Service('em_spco_formation/data', spcof_data, self.run_server)

        rospy.loginfo("[Service spcof/data] Ready em_spco_formation/data")


if __name__ == "__main__":

    rospy.init_node('spcof_data_server')
    TRIALNAME = rospy.get_param('~trial_name')

    hoge = DataServer()

    rospy.spin()

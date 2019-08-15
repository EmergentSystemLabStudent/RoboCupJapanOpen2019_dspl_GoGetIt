#! /usr/bin/env python

from __init__ import *

import subprocess

from darknet_ros_msgs.msg import BoundingBoxes

class GetYoloFeature():

    # service for saving image feature
    def yolo_server(self,req):

        count = req.count

        yolo_BoW = np.zeros(self.yolo_length)
        for x in xrange(len(self.bounding_boxes)):
            yolo_BoW[self.yolo_detection_classes.index(self.bounding_boxes[x].Class)] += 1

        fp = open(self.DATA_FOLDER + "/image.csv",'a')
        fp.write(','.join(map(str, yolo_BoW)))
        fp.write('\n')
        fp.close()
        rospy.loginfo("[Service spcof/yolo] save new BoW")

        return spcof_yoloResponse(True)

    # hold yolo
    def yolo_callback(self,data):

        self.bounding_boxes = data.bounding_boxes

    def __init__(self):

        rospy.Subscriber(YOLO_TOPIC, BoundingBoxes, self.yolo_callback, queue_size=1)

        self.DATA_FOLDER = DATASET_FOLDER + TRIALNAME

        s = rospy.Service('em_spco_formation/data/yolo', spcof_yolo, self.yolo_server)
        rospy.loginfo("[Service spcof/yolo] Ready em_spco_formation/yolo")

        self.yolo_detection_classes = rospy.get_param("~/darknet_ros/yolo_model/detection_classes/names")
        self.yolo_length = len(self.yolo_detection_classes)


if __name__ == "__main__":

    rospy.init_node('spcof_yolo_server')
    TRIALNAME = rospy.get_param('~trial_name')

    hoge = GetYoloFeature()

    rospy.spin()

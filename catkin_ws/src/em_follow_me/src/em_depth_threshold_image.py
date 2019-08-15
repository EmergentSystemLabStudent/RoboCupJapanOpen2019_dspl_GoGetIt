#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

class DepthThresholdImage(object):

    def depthImageCallback(self,data):

        depthImage = CvBridge().imgmsg_to_cv2(data, '16UC1')
        colorImage = CvBridge().imgmsg_to_cv2(self.colorImageData, 'rgb8')

        colorImage[depthImage<10] = [0,0,0]
        colorImage[depthImage>self.depthThreshold] = [0,0,0]

        self.image_pub.publish(CvBridge().cv2_to_imgmsg(colorImage, "rgb8"))

    def colorImageCallback(self,data):

        self.colorImageData = data

    def __init__(self):

        self.depthThreshold = rospy.get_param('~depth_threshold')

        rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw", Image, self.colorImageCallback, queue_size=1)
        rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_raw", Image, self.depthImageCallback, queue_size=1)

        self.image_pub = rospy.Publisher("/depth_threshold_image/image", Image, queue_size=1)


if __name__ == '__main__':

    rospy.init_node('DepthThresholdImage', anonymous=True)
    hoge = DepthThresholdImage()
    rospy.spin()

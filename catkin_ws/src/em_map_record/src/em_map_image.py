#! /usr/bin/env python

import rospy
from em_map_record.srv import *
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class GetImage():
    #save image
    def image_server(self, req):
        count = req.count
        image_name = self.DATA_FOLDER + "/image/" + str(count) + ".png"
        cv2.imwrite(image_name, self.frame)
        rospy.loginfo("[Service em_map_record/image] save new image as %s", image_name)
        return map_imageResponse(True)

    def image_callback(self, image):
        bridge = CvBridge()
        try:
            self.frame = bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError, e:
            print e

    def __init__(self):
        IMAGE_TOPIC = rospy.get_param('~image_topic')
        TASKNAME = rospy.get_param('~task_name')
        self.DATA_FOLDER = "../data/" + TASKNAME
        s = rospy.Service('em_map_record/image', map_image, self.image_server)
        rospy.loginfo("[Service em_map_record/image] Ready em_map_record/image")

        rospy.Subscriber(IMAGE_TOPIC, Image, self.image_callback, queue_size=1)

if __name__ == "__main__":
    rospy.init_node('em_map_image')
    hoge = GetImage()
    rospy.spin()

#! /usr/bin/env python

from __init__ import *

import subprocess

import glob
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import caffe

class GetImageFeature():

    # service for saving image feature
    def image_server(self, req):

        count = req.count

        scores = self.net.predict([self.frame])
        feat = self.net.blobs[self.layer].data
        
        feat = np.average(feat, axis=0)
        fp = open(self.DATA_FOLDER + "/image.csv",'a')
        fp.write(','.join(map(str, feat)))
        fp.write('\n')
        fp.close()

        image_name = self.DATA_FOLDER + "/image/" + str(count) + ".jpg"
        cv2.imwrite(image_name, self.frame)
        rospy.loginfo("[Service spcof/image] save new image as %s", image_name)

        return spcof_imageResponse(True)

    # hold image
    def image_callback(self, image):

        bridge = CvBridge()
        try:
            self.frame = bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError, e:
            print e

    def __init__(self):

        rospy.Subscriber(IMAGE_TOPIC, Image, self.image_callback, queue_size=1)
        
        self.MEAN_FILE = CAFFE_FOLDER + "python/caffe/imagenet/ilsvrc_2012_mean.npy"
        self.MODEL_FILE = CAFFE_FOLDER + "models/bvlc_reference_caffenet/deploy.prototxt"
        self.PRETRAINED = CAFFE_FOLDER + "models/bvlc_reference_caffenet/bvlc_reference_caffenet.caffemodel"
        self.layer = 'prob' #'fc6wi'

        self.net = caffe.Classifier(self.MODEL_FILE, self.PRETRAINED)
        caffe.set_mode_cpu()
        self.net.transformer.set_mean('data', np.load(self.MEAN_FILE))
        self.net.transformer.set_raw_scale('data', 255)
        self.net.transformer.set_channel_swap('data', (2,1,0))

        self.DATA_FOLDER = DATASET_FOLDER + TRIALNAME

        s = rospy.Service('em_spco_formation/data/image', spcof_image, self.image_server)
        rospy.loginfo("[Service spcof/image] Ready em_spco_formation/image")

if __name__ == "__main__":

    rospy.init_node('spcof_image_server')
    TRIALNAME = rospy.get_param('~trial_name')

    hoge = GetImageFeature()

    rospy.spin()

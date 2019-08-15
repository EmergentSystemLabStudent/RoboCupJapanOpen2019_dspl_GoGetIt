#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import numpy as np

from em_spco_formation.srv import *

TRIALNAME = "trial"

IMAGE_TOPIC = "/hsrb/head_rgbd_sensor/rgb/image_raw"
YOLO_TOPIC = "/darknet_ros/bounding_boxes"
VOCAB_TOPIC = "/em/spco_formation/vocab"
POSE_TOPIC = "/global_pose"

DATASET_FOLDER = "../training_data/"
CAFFE_FOLDER = "/opt/caffe/"
RESULT_PATH = "../result/"

ITERATION = 500
K0 = 0.01
V0 = 10000      #楕円の大きさ
GAMMA = 100.0   #カテゴリの分かれやすさ
BETA = 0.1      #単語
CHI = 100.0     #画像
ALPHA = 0.05    #ガウス分布ノイズ

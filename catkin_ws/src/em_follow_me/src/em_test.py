#! /usr/bin/env python

import rospy
import subprocess, time
from time import sleep
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from sklearn.ensemble import RandomForestClassifier
from sklearn.cross_validation import train_test_split

from darknet_ros_msgs.msg import BoundingBoxes

import matplotlib.pyplot as plt

color = [   '#ffffff', '#f0f000', '#ffc0ff', '#c0c000', '#c05800', '#00ffff', '#400000', '#ff0000', '#004000', '#ff00ff',
            '#707070', '#404000', '#c0ffff', '#c000c0', '#b0b0b0', '#ffff40', '#d0d0d0', '#303030', '#ffc0c0', '#008000',
            '#909090', '#000040', '#401800', '#8080ff', '#400040', '#c0ffc0', '#40ff40', '#ff8080', '#00c0c0', '#101010',
            '#c0c0ff', '#00e000', '#80ffff', '#004040', '#0000ff', '#c00000', '#505050', '#ffe080', '#ffffa0', '#0000c0',
            '#ffc040', '#f0f0f0', '#ff8000', '#ff80ff']

DATA_SIZE = 201
DATA_LEN = 8
MAX_SPEED = 0.2
F = np.array([[1.0, 0.0],[0.0, 1.0]])

class GetURG(object):

    def clustering(self, urg_deg, size):

        cluster = np.arange(size)

        for x in xrange(size-1):
            y = urg_deg[x+1:min(x+70,size)]
            d = np.sqrt(urg_deg[x,1] ** 2 + y[:,1] ** 2 - 2 * urg_deg[x,1] * y[:,1] * np.cos(urg_deg[x,0] - y[:,0]))
            same_class = cluster[x+1:min(x+70,size)][d <= 0.06]
            for i in same_class:
                cluster[cluster==i] = cluster[x]

        cluster_num = 1
        for x in np.unique(cluster):
            if np.sum(cluster==x) > 5:
                cluster[cluster==x] = cluster_num
                cluster_num += 1
            else :
                cluster[cluster==x] = 0

        means = np.empty((cluster_num,2))
        for x in xrange(cluster_num):
            means[x] = np.mean(urg_deg[cluster==x], axis=0)

        return cluster, means, cluster_num

    def RF_data(self, urg_deg, cluster, means, cluster_num):

        output = np.empty((cluster_num-1,DATA_LEN-1))
        for x in xrange(1,cluster_num):
            output[x-1,0] = np.sum(cluster==x)
            output[x-1,1:3] = means[x]
            output[x-1,3:5] = np.var(urg_deg[cluster==x], axis=0)
            output[x-1,5] = np.min(urg_deg[cluster==x], axis=0)[1]
            output[x-1,6] = np.max(urg_deg[cluster==x], axis=0)[1]
        return output

    def KF(self, output):

        self.delta_t = time.clock() - self.pre_time
        
        if output[1][DATA_LEN] >= 0.7:
            position = np.array([(output[0,1] + output[1,1]) / 2.0, (output[0,2] + output[1,2]) / 2.0])
        elif output[0][DATA_LEN] >= 0.7:
            position = np.array([output[0,1], output[0,2]])
        else :
            position = np.array([0.0, 0.0])

        if not self.init_flag and output[0][DATA_LEN] >= 0.7:
            self.pre_pos = position
            self.init_flag = True
            self.pre_time = time.clock()
        elif output[0][DATA_LEN] >= 0.7:

            C = np.array([[output[0,3], 0.0],[0.0, output[0,4]]])

            mu_ = np.dot(F, self.pre_pos - np.array([self.pre_vel.linear.x, self.pre_vel.angular.z]) * self.delta_t)
            sigma_ = np.dot(F, np.dot(self.pre_sigma, F.T))
            K1 = sigma_ * C.T * np.linalg.inv(np.dot(C, np.dot(sigma_ + np.ones((2,2))*100, C.T)))
            K2 = sigma_ * C.T * np.linalg.inv(sigma_)
            mu = mu_ + np.dot(K1, (np.dot(C, position) - np.dot(C, mu_)))
            sigma = sigma_ - np.dot(K2, np.dot(C, sigma_))

            position = mu
            self.pre_sigma = sigma
            self.pre_pos = mu
            self.pre_time = time.clock()
        else:
            position = self.pre_pos - np.array([self.pre_vel.angular.z, self.pre_vel.linear.x]) * self.delta_t

        return position

    def tracePosition(self, position, urg_deg, size):

        vel = Twist()

        if position[1] > 0.8:
            vel.linear.x = min(self.pre_vel.linear.x + 0.02, position[1] * 0.1 + 0.1, MAX_SPEED)
        elif position[1] < 0.5:
            vel.linear.x = -0.1
        else :
            vel.linear.x = max(self.pre_vel.linear.x - 0.01 , 0)
        
        if position[0] > 0.15:
            vel.angular.z = 0.8
        elif position[0] < -0.15:
            vel.angular.z = -0.8
        
        if np.sum(urg_deg[:size/3,1] < 0.4) > 5:
            vel.linear.y = 0.15
        elif np.sum(urg_deg[size*2/3:,1] < 0.4) > 5:
            vel.linear.y = -0.15

        vel.linear.x = 0 # test
        vel.linear.y = 0 # test
        vel.angular.z = 0 # test
        
        self.vel_pub.publish(vel)
        self.pre_vel = vel

    # trial mode
    def urgCallback(self, hoge):

        if not self.pause_flag:
            return
        
        ranges = hoge.ranges
        angle_min = hoge.angle_min
        angle_increment = hoge.angle_increment

        # clustering
        size = len(ranges)
        angle = angle_min
        urg_deg = np.array([[angle + angle_increment * x, ranges[x]] for x in range(size)])
        urg_xy = np.array([np.cos(urg_deg[:,0]) * urg_deg[:,1], np.sin(urg_deg[:,0]) * urg_deg[:,1]])
        cluster, means, cluster_num = self.clustering(urg_deg, size)

        # random forest classifier
        output = self.RF_data(urg_deg, cluster, means, cluster_num)
        prediction = self.clf.predict_proba(output)
        output = np.hstack((output, prediction))
        output = output[output[:,DATA_LEN].argsort()[::-1], :]

        # Kalman filter
        position = self.KF(output[0:2])

        # output by matplotlib
        plt.clf()
        plt.xlim([-2,4])
        plt.ylim([-2,4])
        for x in xrange(size):
            plt.scatter(urg_xy[0,x], urg_xy[1,x], c = color[cluster[x]], edgecolor = color[cluster[x]])
            # plt.scatter(np.cos(urg_deg[x][0]) * urg_deg[x][1], np.sin(urg_deg[x][0]) * urg_deg[x][1], c = color[cluster[x]], edgecolor = color[cluster[x]])
        plt.scatter(np.cos(output[0,1]) * output[0,2], np.sin(output[0,1]) * output[0,2], c = "r", edgecolor = "k")
        plt.scatter(np.cos(output[1,1]) * output[1,2], np.sin(output[1,1]) * output[1,2], c = "r", edgecolor = "k")
        plt.scatter(np.cos(position[0]) * position[1], np.sin(position[0]) * position[1], c = "b", edgecolor = "k")
        plt.pause(0.01)

        # move
        self.tracePosition(position, urg_deg, size)

        print self.count
        self.count += 1

    def pauseCallback(self, msg):
        
        if msg.data == True:
            self.pause_flag = True
        elif msg.data == False:
            self.pause_flag = False
            self.init_flag = False

    def depthImageCallback(self, msg):
        
        self.depthImage = np.array(msg.data)

    def detectCallback(self, msg):

        person_depth = []
        other_depth = []
        person_range = []
        other_range = []
        for x in xrange(len(msg.bounding_boxes)):

            hoge = msg.bounding_boxes[x]

            if msg.bounding_boxes[x].Class == "people":
                print msg.bounding_boxes[x].probability
                print msg.bounding_boxes[x].xmin, msg.bounding_boxes[x].xmax
                print msg.bounding_boxes[x].ymin, msg.bounding_boxes[x].ymax
                person_depth = self.depthImage[(hoge.xmin + hoge.xmax)/2, (hoge.ymin + hoge.ymax)/2]
                person_range = (hoge.xmin + hoge.xmax) / 2 / len(self.depthImage)
            else:
                other_depth = self.depthImage[(hoge.xmin + hoge.xmax)/2, (hoge.ymin + hoge.ymax)/2]
                other_range = (hoge.xmin + hoge.xmax) / 2 / len(self.depthImage)

        self.yolo_flag = True

    def __init__(self):

        self.init_flag = False
        self.count = 1
        self.pre_pos = np.zeros(2)
        self.pre_sigma = np.array([[1.0,0.0],[0.0,1.0]])
        self.pre_vel = Twist()
        self.pre_time = 0
        self.pause_flag = False

        self.depthImage = np.empty((480,640))
        self.yolo_flag = False

        # trial mode
        self.input = np.zeros((2,DATA_LEN))
        for x in xrange(DATA_SIZE):
            hoge = np.loadtxt("dataset_all/output" + str(x+1) + ".csv", delimiter=",")
            self.input = np.append(self.input, hoge, axis=0)
        self.input = np.delete(self.input, [0,1], 0)

        train_X = np.delete(self.input, DATA_LEN-1, 1)
        train_y = self.input[:,DATA_LEN-1]

        # (train_X, test_X ,train_y, test_y) = train_test_split(train_X, train_y, test_size = 0.3, random_state = 0)
        self.clf = RandomForestClassifier(random_state=0)
        self.clf = self.clf.fit(train_X, train_y)
        # print('Train score: {}'.format(self.clf.score(train_X, train_y)))
        # print('Test score: {}'.format(self.clf.score(test_X, test_y)))

        # save dataset
        # p = subprocess.Popen("rm -rf " + "dataset", shell=True)
        # sleep(1.0)
        # p = subprocess.Popen("mkdir -p " + "dataset", shell=True)

        self.vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
        rospy.Subscriber("/hsrb/base_scan", LaserScan, self.urgCallback, queue_size = 1)
        rospy.Subscriber("/em/follow_me", Bool, self.pauseCallback, queue_size = 1)
        rospy.Subscriber("/yolo2_node/detections", BoundingBoxes, self.detectCallback, queue_size = 1)
        rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", Image, self.depthImageCallback, queue_size = 1)

if __name__ == '__main__':

    rospy.init_node('GetURG', anonymous=True)
    hoge = GetURG()
    rospy.spin()

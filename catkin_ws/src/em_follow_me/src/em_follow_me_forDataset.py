#! /usr/bin/env python

import rospy
import subprocess, time
from time import sleep
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

from sklearn.ensemble import RandomForestClassifier
from sklearn.cross_validation import train_test_split

import matplotlib.pyplot as plt

color = [   '#ffffff', '#f0f000', '#ffc0ff', '#c0c000', '#c05800', '#00ffff', '#400000', '#ff0000', '#004000', '#ff00ff',
            '#707070', '#404000', '#c0ffff', '#c000c0', '#b0b0b0', '#ffff40', '#d0d0d0', '#303030', '#ffc0c0', '#008000',
            '#909090', '#000040', '#401800', '#8080ff', '#400040', '#c0ffc0', '#40ff40', '#ff8080', '#00c0c0', '#101010',
            '#c0c0ff', '#00e000', '#80ffff', '#004040', '#0000ff', '#c00000', '#505050', '#ffe080', '#ffffa0', '#0000c0',
            '#ffc040', '#f0f0f0', '#ff8000', '#ff80ff']

DATA_SIZE = 102
DATA_LEN = 8

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

    # save dataset mode
    def urgCallback(self, hoge):

        ranges = hoge.ranges
        angle_min = hoge.angle_min
        angle_increment = hoge.angle_increment

        # clustering
        size = len(ranges)
        angle = angle_min
        urg_deg = np.array([[angle + angle_increment * x, ranges[x]] for x in range(size)])
        cluster, means, cluster_num = self.clustering(urg_deg, size)

        # random forest classifier
        output = self.RF_data(urg_deg, cluster, means, cluster_num)
        prediction = self.clf.predict_proba(output)
        output = np.hstack((output, prediction))
        output = output[output[:,DATA_LEN].argsort()[::-1], :]

        # make output data
        output = np.delete(output, DATA_LEN-1, 1)
        for x in xrange(len(output)):
            if output[x,DATA_LEN-1] >= 0.5:
                output[x,DATA_LEN-1] = 1.0
            else :
                output[x,DATA_LEN-1] = 0.0

        # output by matplotlib
        plt.clf()
        plt.xlim([-2,2])
        plt.ylim([-2,2])
        for x in xrange(size):
            plt.scatter(np.cos(urg_deg[x][0]) * urg_deg[x][1], np.sin(urg_deg[x][0]) * urg_deg[x][1], c = color[cluster[x]], edgecolor = color[cluster[x]])
        plt.scatter(0, 0, c = "r", edgecolor = "k")
        for x in xrange(len(output)):
            if output[x,0] > 5:
                plt.scatter(np.cos(output[x,1]) * output[x,2], np.sin(output[x,1]) * output[x,2], marker = '$'+str(x)+'$')
        plt.pause(0.01)

        plt.savefig("dataset/" + str(self.count) + ".png")
        np.savetxt("dataset/output" + str(self.count) + ".csv", output, delimiter=",")

        self.count += 1
        print self.count

    def __init__(self):

        self.count = 1

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
        p = subprocess.Popen("rm -rf " + "dataset", shell=True)
        sleep(1.0)
        p = subprocess.Popen("mkdir -p " + "dataset", shell=True)

        rospy.Subscriber("/hsrb/base_scan", LaserScan, self.urgCallback, queue_size = 1)

if __name__ == '__main__':

    rospy.init_node('GetURG', anonymous=True)
    hoge = GetURG()
    rospy.spin()

#! /usr/bin/env python

from __init__ import *

import glob
import re
import numpy as np
from scipy.stats import multivariate_normal

class Name_to_Place():

    def name2place_server(self, req):

        mu = np.loadtxt(self.RESULT_FOLDER + "/mu.csv")
        word = np.loadtxt(self.RESULT_FOLDER + "/word.csv")
        pi = np.loadtxt(self.RESULT_FOLDER + "/pi.csv")

        train_word = np.loadtxt(self.DATA_FOLDER + "/word.csv",delimiter=",")

        sigma = []
        file = glob.glob(self.RESULT_FOLDER + "/sigma/*.csv")
        convert = lambda text: int(text) if text.isdigit() else text
        alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
        file.sort(key = alphanum_key)
        for f in file:
            hoge = np.loadtxt(f)
            sigma.append(hoge)

        f = open(self.DATA_FOLDER + "/word_list.csv")
        hoge = f.read()
        f.close()
        word_list = hoge.split('\n')

        word_class = -1
        for x in xrange(len(word_list)):
            if word_list[x] == req.place_name:
                word_class = x
        if word_class == -1:
            rospy.loginfo("[Service spcof/name2place] I don't know that place.")
            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 0
            return spcof_name2placeResponse(pose, False)

        prob = np.array([0.0 for k in xrange(len(word))])
        for x in xrange(len(mu)):
            prob[x] = word[x][word_class] * pi[x]

        c = np.argmax(prob)


        hoge = multivariate_normal.rvs(mean = mu[c], cov = sigma[c], size = 1)
        pose = Pose()
        pose.position.x = hoge[0]
        pose.position.y = hoge[1]
        pose.orientation.z = hoge[2] / (2 * np.sqrt(abs(hoge[3])))
        pose.orientation.w = np.sqrt(abs(hoge[3]))


        return spcof_name2placeResponse(pose, True)

    def __init__(self):

        self.DATA_FOLDER = DATASET_FOLDER + TRIALNAME
        self.RESULT_FOLDER = RESULT_PATH + TRIALNAME

        s = rospy.Service('em_spco_formation/name2place', spcof_name2place, self.name2place_server)

if __name__ == "__main__":

    rospy.init_node('spcof_name2place_server')
    TRIALNAME = rospy.get_param('~trial_name')

    hoge = Name_to_Place()

    rospy.spin()

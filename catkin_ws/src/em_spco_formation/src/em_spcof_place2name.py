#! /usr/bin/env python

from __init__ import *

import glob, re
from scipy.stats import multivariate_normal

class Place_to_Name():

    def place2name_server(self, req):

        mu = np.loadtxt(self.RESULT_FOLDER + "/mu.csv")
        word = np.loadtxt(self.RESULT_FOLDER + "/word.csv")
        pi = np.loadtxt(self.RESULT_FOLDER + "/pi.csv")

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

        cordinate = [float(req.point.x),float(req.point.y),0,0]
        r_prob = np.ones(len(mu))
        for x in xrange(-10,10):
            cordinate[2] = x * 0.1
            for y in xrange(-10,10):
                cordinate[3] = y * 0.1
                for z in xrange(len(mu)):
                    r_prob[z] = multivariate_normal.pdf(cordinate, mean=mu[z], cov=sigma[z])
                    r_prob[z] = r_prob[z] * pi[z]

        word_prob = np.zeros(len(word[0]))
        for x in xrange(len(mu)):
            for y in xrange(len(word)):
                for z in xrange(len(word[0])):
                    word_prob[z] += word[y][z] * pi[x] * r_prob[x]

        hoge = word_list[np.argmax(word_prob)]
        rospy.loginfo("place name = %s", hoge)
        print word_prob

        return spcof_place2nameResponse(hoge, True)

    def __init__(self):

        s = rospy.Service('em_spco_formation/place2name', spcof_place2name, self.place2name_server)

        self.DATA_FOLDER = DATASET_FOLDER + TRIALNAME
        self.RESULT_FOLDER = RESULT_PATH + TRIALNAME

        rospy.loginfo("[Service spcof/place2name] Ready em_spco_formation/place2name")

if __name__ == "__main__":

    rospy.init_node('spcof_place2name_server')
    TRIALNAME = rospy.get_param('~trial_name')

    hoge = Place_to_Name()

    rospy.spin()

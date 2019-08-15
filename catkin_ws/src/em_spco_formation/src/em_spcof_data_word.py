#! /usr/bin/env python

from __init__ import *

# import numpy as np

class GetWordFeature():

    # service for saving word
    def word_server(self, req):

        place_name = req.sentence.split()
        count = req.count
        # if len(place_name) == 0:
        #     return spcof_wordResponse(False)
        for x in xrange(len(place_name)):
            if place_name[x] not in self.place_name_list:
                self.place_name_list.append(place_name[x])
        place_name = np.array([place_name.count(self.place_name_list[i]) for i in range(len(self.place_name_list))])

        try:
            word = np.loadtxt(self.DATA_FOLDER + "/word.csv", delimiter=",")
            hoge = np.zeros([count+1,len(self.place_name_list)])
            for x in xrange(count):
                if len(np.shape(word)) == 2:
                    hoge[x,0:len(word[0])] = word[x,0:len(word[0])]
                elif len(np.shape(word)) == 1:
                    if count == 1:
                        hoge[0,0:len(word)] = word[0:len(word)]
                    else:
                        hoge[x,0] = word[x]
                else:
                    hoge[0,0] = word
            word = hoge
            for x in xrange(len(place_name)):
                word[count,x] = place_name[x]
        except Exception as e:
            word = np.array([place_name])


        np.savetxt(self.DATA_FOLDER + "/word.csv", word, fmt="%.0f", delimiter=",")
        fp = open(self.DATA_FOLDER + "/word_list.csv", 'w')
        for x in xrange(len(self.place_name_list)):
            fp.write(self.place_name_list[x] + "\n")
        fp.close()

        rospy.loginfo("[Service spcof/word] get new word : %s", req.sentence)

        return spcof_wordResponse(True)

    def __init__(self):

        self.DATA_FOLDER = DATASET_FOLDER + TRIALNAME
        self.place_name_list = []
        try:
            with open(self.DATA_FOLDER + "/word_list.csv") as f:
                self.place_name_list = [s.strip() for s in f.readlines()]
        except:
            self.place_name_list = []

        s = rospy.Service('em_spco_formation/data/word', spcof_word, self.word_server)
        rospy.loginfo("[Service spcof/word] Ready em_spco_formation/word")

if __name__ == "__main__":

    rospy.init_node('spcof_word_server')
    TRIALNAME = rospy.get_param('~trial_name')

    hoge = GetWordFeature()

    rospy.spin()

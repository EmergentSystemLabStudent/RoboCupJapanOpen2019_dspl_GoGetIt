#! /usr/bin/env python

from __init__ import *

import glob, re, math
from scipy.stats import multivariate_normal
from visualization_msgs.msg import Marker, MarkerArray

color = [[0.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0],[1.0,0.0,0.0],[1.0,1.0,0.0],[1.0,0.0,1.0],
[0.0,1.0,1.0],[0.0,1.0,0.5],[0.0,0.5,1.0],[0.5,1.0,0.0],[1.0,0.5,0.0],[0.5,0.0,1.0],[1.0,0.0,0.5],
[0.5,0.0,0.5],[0.5,0.5,0.0],[0.0,0.5,0.5],[0.0,0.5,0.0],[0.0,0.0,0.5],[0.5,0.0,0.0]]

class Show_Rviz():

    def show_rviz_server(self, req):

        if not req.start:
            return spcof_rvizResponse(False)

        mu = np.loadtxt(self.RESULT_FOLDER + "/mu.csv")
        word = np.loadtxt(self.RESULT_FOLDER + "/word.csv")
        pi = np.loadtxt(self.RESULT_FOLDER + "/pi.csv")
        Ct = np.loadtxt(self.RESULT_FOLDER + "/Ct.csv")

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

        marker_array = MarkerArray()
        rviz_id_count = 0
        data_size = len(mu)

        for i in xrange(data_size):

            if Ct[i] > 0:

                sig_marker = Marker()
                sig_marker.type= Marker.CYLINDER
                (eigValues,eigVectors) = np.linalg.eig(sigma[i])
                angle = (math.atan2(eigVectors[1, 0], eigVectors[0, 0]))
                sig_marker.scale.x = 10*math.sqrt(eigValues[0])
                sig_marker.scale.y = 10*math.sqrt(eigValues[1])
                sig_marker.scale.z = 0.6 * word[i][np.argsort(word[i])[::-1][0]]
                sig_marker.pose.position.x = mu[i][0]
                sig_marker.pose.position.y = mu[i][1]
                sig_marker.pose.position.z = 0.3 * word[i][np.argsort(word[i])[::-1][0]]
                sig_marker.pose.orientation.w = math.cos(angle*0.5)
                sig_marker.pose.orientation.z = math.sin(angle*0.5)
                sig_marker.header.frame_id = 'map'
                sig_marker.header.stamp = rospy.get_rostime()
                sig_marker.id = rviz_id_count
                sig_marker.action = Marker.ADD
                sig_marker.color.r = color[i+1][0]
                sig_marker.color.g = color[i+1][1]
                sig_marker.color.b = color[i+1][2]
                sig_marker.color.a = 0.6
                marker_array.markers.append(sig_marker)
                rviz_id_count += 1

        height = 0
        for i in xrange(len(mu)):

            if Ct[i] > 0:

                if len(word_list) - 1 >= 3:
                    output_text = "%s: %.1f\n%s: %.1f\n%s: %.1f" %(word_list[np.argsort(word[i])[::-1][0]], word[i][np.argsort(word[i])[::-1][0]]*100, word_list[np.argsort(word[i])[::-1][1]], word[i][np.argsort(word[i])[::-1][1]]*100, word_list[np.argsort(word[i])[::-1][2]], word[i][np.argsort(word[i])[::-1][2]]*100)
                elif len(word_list) - 1 == 2:
                    output_text = "%s: %.1f\n%s: %.1f" %(word_list[np.argsort(word[i])[::-1][0]], word[i][np.argsort(word[i])[::-1][0]]*100, word_list[np.argsort(word[i])[::-1][1]], word[i][np.argsort(word[i])[::-1][1]]*100)
                elif len(word_list) - 1 == 1:
                    output_text = "%s: %.1f" %(word_list[np.argsort(word[i])[::-1][0]], word[i][np.argsort(word[i])[::-1][0]]*100)
                else:
                    output_text = "ERROR"

                text_marker = Marker()
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.header.frame_id = 'map'
                text_marker.ns = "marker" + str(rviz_id_count)
                text_marker.header.stamp = rospy.get_rostime()
                text_marker.id = rviz_id_count
                text_marker.action = Marker.ADD
                text_marker.scale.x = 0.3
                text_marker.scale.y = 0.3
                text_marker.scale.z = 0.3
                text_marker.pose.position.x = mu[i][0]
                text_marker.pose.position.y = mu[i][1]
                text_marker.pose.position.z = 0.6 * word[i][np.argsort(word[i])[::-1][0]] + 0.4
                text_marker.color.r = color[i+1][0]
                text_marker.color.g = color[i+1][1]
                text_marker.color.b = color[i+1][2]
                text_marker.color.a = 0.8
                text_marker.text = output_text
                marker_array.markers.append(text_marker)
                rviz_id_count += 1
                height += 1

        self.pub_rviz.publish(marker_array)
        rospy.sleep(0.01)

        return spcof_rvizResponse(True)

    def __init__(self):

        s = rospy.Service('em_spco_formation/rviz', spcof_rviz, self.show_rviz_server)
        self.pub_rviz = rospy.Publisher('/em/draw_position/array', MarkerArray, queue_size = 1)

        self.DATA_FOLDER = DATASET_FOLDER + TRIALNAME
        self.RESULT_FOLDER = RESULT_PATH + TRIALNAME

        rospy.loginfo("[Service spcof/rviz] Ready em_spco_formation/rviz")

if __name__ == "__main__":

    rospy.init_node('spcof_rviz')
    TRIALNAME = rospy.get_param('~trial_name')

    hoge = Show_Rviz()

    rospy.spin()

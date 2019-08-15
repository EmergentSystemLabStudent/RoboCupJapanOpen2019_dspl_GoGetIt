#! /usr/bin/env python

from __init__ import *

from geometry_msgs.msg import PoseStamped

class GetPose():

    # service for saving pose
    def pose_server(self, req):

        count = req.count

        fp = open(self.DATA_FOLDER + "/pose.csv",'a')
        fp.write(str(self.pose.x) + "," + str(self.pose.y) + "," + str(self.sin) + "," + str(self.cos) + "\n")
        fp.close()

        # for robocup2019
        fp = open(self.DATA_FOLDER + "/pose_wrs.csv",'a')
        fp.write(str(self.pose.x) + "," + str(self.pose.y) + "," + str(self.orientation.z) + "," + str(self.orientation.w) + "\n")
        fp.close()
        ###

        rospy.loginfo("[Service spcof/pose] get new pose (%f,%f,%f,%f)", self.pose.x, self.pose.y, self.sin, self.cos)

        return spcof_poseResponse(True)

    # hold self pose
    def pose_callback(self, hoge):

        self.pose = hoge.pose.position
        self.orientation = hoge.pose.orientation

        self.sin = 2 * self.orientation.w * self.orientation.z
        self.cos = self.orientation.w * self.orientation.w - self.orientation.z * self.orientation.z

    def __init__(self):

        rospy.Subscriber(POSE_TOPIC, PoseStamped, self.pose_callback, queue_size=1)

        self.DATA_FOLDER = DATASET_FOLDER + TRIALNAME

        s = rospy.Service('em_spco_formation/data/pose', spcof_pose, self.pose_server)
        rospy.loginfo("[Service spcof/pose] Ready em_spco_formation/pose")

if __name__ == "__main__":

    rospy.init_node('spcof_pose_server')
    TRIALNAME = rospy.get_param('~trial_name')

    hoge = GetPose()

    rospy.spin()

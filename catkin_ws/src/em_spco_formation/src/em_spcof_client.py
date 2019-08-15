#!/usr/bin/env python

from __init__ import *

def example_learn_client(start):

    fin = False
    rospy.wait_for_service('em_spco_formation/learn')

    while fin == False:
        try:
            em_spco_formation = rospy.ServiceProxy('em_spco_formation/learn', spcof_learn)
            res = em_spco_formation(start)
            fin = res.finish
        except rospy.ServiceException, e:
            rospy.loginfo("[Service spcof/client] call failed: %s", e)

    return res.concept_num

def example_data_client(sentence):

    fin = False
    rospy.wait_for_service('em_spco_formation/data')

    while fin == False:
        try:
            em_spco_formation = rospy.ServiceProxy('em_spco_formation/data', spcof_data)
            res = em_spco_formation(sentence)
            fin = res.finish
        except rospy.ServiceException, e:
            rospy.loginfo("[Service spcof/client] call failed: %s", e)

    return res.data_num

def example_rviz_client(hoge):

    fin = False
    rospy.wait_for_service('em_spco_formation/rviz')

    while fin == False:
        try:
            em_spco_formation = rospy.ServiceProxy('em_spco_formation/rviz', spcof_rviz)
            res = em_spco_formation(hoge)
            fin = res.done
        except rospy.ServiceException, e:
            rospy.loginfo("[Service spcof/client] call failed: %s", e)

    return "end"

def example_name2place_client(place_name):

    fin = False
    rospy.wait_for_service('em_spco_formation/name2place')

    while fin == False:
        try:
            em_spco_formation = rospy.ServiceProxy('em_spco_formation/name2place', spcof_name2place)
            res = em_spco_formation(place_name)
            fin = res.done
        except rospy.ServiceException, e:
            rospy.loginfo("[Service spcof/client] call failed: %s", e)

    return res.pose

def example_place2name_client(point):

    fin = False
    rospy.wait_for_service('em_spco_formation/place2name')

    while fin == False:
        try:
            em_spco_formation = rospy.ServiceProxy('em_spco_formation/place2name', spcof_place2name)
            res = em_spco_formation(point)
            fin = res.done
        except rospy.ServiceException, e:
            rospy.loginfo("[Service spcof/client] call failed: %s", e)

    return res.place_name

if __name__ == "__main__":

    rospy.init_node('spcof_client')

    # class_size = 0
    # s = "table"
    # class_size = example_data_client(s)
    # if class_size != 0:
    #     rospy.loginfo("[Service spcof/data] %s", s)
    # rospy.sleep(1.0)
    # s = "table"
    # class_size = example_data_client(s)
    # if class_size != 0:
    #     rospy.loginfo("[Service spcof/data] %s", s)
    # rospy.sleep(1.0)
    # s = "kitchen"
    # class_size = example_data_client(s)
    # if class_size != 0:
    #     rospy.loginfo("[Service spcof/data] %s", s)
    # rospy.sleep(1.0)

    hoge = example_learn_client(4)
    rospy.loginfo("[Service spcof/learn] number of concept is %d", hoge)
    hoge = example_rviz_client(True)
    rospy.loginfo("[Service spcof/rviz] %s", hoge)

    # hoge = example_name2place_client("desk")
    # rospy.loginfo("[Service spcof/name2place] %f, %f, %f, %f", hoge.position.x, hoge.position.y, hoge.orientation.z, hoge.orientation.w)

    # point = Point()
    # point.x = 0.0
    # point.y = 0.0
    # hoge = example_place2name_client(point)
    # rospy.loginfo("[Service spcof/place2name] %s", hoge)

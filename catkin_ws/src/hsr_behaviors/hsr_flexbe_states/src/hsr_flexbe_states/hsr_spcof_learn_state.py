#!/usr/bin/env python
import rospy
from em_spco_formation.srv import *
from flexbe_core import EventState, Logger

class hsr_SpcofLearn(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    #> spcof_text       string  speech text

    <= completed        completed
    '''

    def __init__(self):
        super(hsr_SpcofLearn,self).__init__(outcomes=['completed'],input_keys=['request'],output_keys=['spcof_text'])
        self.em_spcof_learn = rospy.ServiceProxy('em_spco_formation/learn', spcof_learn)
        self.em_spcof_rviz = rospy.ServiceProxy('em_spco_formation/rviz', spcof_rviz)
        self.em_spcof_data = rospy.ServiceProxy('em_spco_formation/data', spcof_data)
        self.sentence = ""

    def execute(self, userdata):
        try:
            teach_count = sum(1 for i in open('/root/HSR/catkin_ws/src/em_spco_formation/training_data/default/pose.csv', 'r'))
        except:
            teach_count = 0
        print "teach_count"
        print teach_count
        try:
            res = self.em_spcof_learn(teach_count)
        except rospy.ServiceException, e:
            rospy.loginfo("[Service spcof/client] call failed: %s", e)
        if res.finish:
            rospy.loginfo("[Service spcof/learn] number of concept is %d", res.concept_num)
            try:
                userdata.spcof_text = "I have finished learning. I can now understand " + str(res.concept_num) + " concepts. I sent the results to my display interface."
                res = self.em_spcof_rviz(True)
            except rospy.ServiceException, e:
                rospy.loginfo("[Service spcof/client] call failed: %s", e)
            if res.done:
                rospy.loginfo("[Service spcof/rviz] publish to rviz")
                return 'completed'

    def on_enter(self, userdata):
        self.sentence = userdata.request
        rospy.wait_for_service('em_spco_formation/learn')
        rospy.wait_for_service('em_spco_formation/rviz')

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

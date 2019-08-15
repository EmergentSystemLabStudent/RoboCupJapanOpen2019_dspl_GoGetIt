#!/usr/bin/env python
import rospy
from em_spco_formation.srv import *
from flexbe_core import EventState, Logger

class hsr_SpcofAddData(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    #> spcof_text       string  speech text

    <= completed        completed
    '''

    def __init__(self):
        super(hsr_SpcofAddData,self).__init__(outcomes=['completed'],input_keys=['request'],output_keys=['spcof_text'])
        self.em_spcof_learn = rospy.ServiceProxy('em_spco_formation/learn', spcof_learn)
        self.em_spcof_rviz = rospy.ServiceProxy('em_spco_formation/rviz', spcof_rviz)
        self.em_spcof_data = rospy.ServiceProxy('em_spco_formation/data', spcof_data)
        self.sentence = ""

    def execute(self, userdata):
        try:
            res = self.em_spcof_data(self.sentence)
        except rospy.ServiceException, e:
            rospy.loginfo("[Service spcof/client] call failed: %s", e)
        if res.data_num != 0:
            rospy.loginfo("[Service spcof/data] %s", self.sentence)
        if res.finish:
            userdata.spcof_text = "I understood!"
        return 'completed'

    def on_enter(self, userdata):
        self.sentence = userdata.request
        rospy.wait_for_service('em_spco_formation/data')

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

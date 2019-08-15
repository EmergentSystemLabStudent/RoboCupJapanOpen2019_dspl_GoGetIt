#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from em_spco_formation.srv import *
from flexbe_core import EventState, Logger

class hsr_SpcofMove(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># move_name    string  goal name

    #> move_text    string  speech text
    #> move_pose    Pose    goal pose
    #> arrive_text  string  speech text after arrive

    <= move         move
    <= failed       cannot move
    '''

    def __init__(self):
        super(hsr_SpcofMove,self).__init__(outcomes=['move','failed'],input_keys=['move_name'],output_keys=['move_text','move_pose','arrive_text'])
        self.em_spco_formation = rospy.ServiceProxy('em_spco_formation/name2place', spcof_name2place)

    def execute(self, userdata):
        try:
            res = self.em_spco_formation(userdata.move_name)
        except rospy.ServiceException, e:
            rospy.loginfo("[Service spcof/client] call failed: %s", e)
        if res.done:
            userdata.move_pose   = res.pose
            userdata.move_text   = "Roger! I go to the " + userdata.move_name + "."
            userdata.arrive_text = "I arrived at the " + userdata.move_name + "."
            return 'move'
        else:
            userdata.move_text = "I don't know the " + userdata.move_name + "."
            return 'failed'

    def on_enter(self, userdata):
        rospy.wait_for_service('em_spco_formation/name2place')

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

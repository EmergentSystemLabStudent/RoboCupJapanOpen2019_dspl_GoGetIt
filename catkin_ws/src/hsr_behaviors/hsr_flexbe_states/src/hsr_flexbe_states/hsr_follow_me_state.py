#!/usr/bin/env python
import rospy, actionlib
from em_follow_me.msg import *
from flexbe_core import EventState, Logger

class hsr_FollowMe(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># request      bool        follow me request

    <= continue     continue

    '''

    def __init__(self):
        super(hsr_FollowMe,self).__init__(outcomes=['continue'],input_keys=['request'])
        self.follow_me_act = actionlib.SimpleActionClient('em_follow_me_action', follow_meAction)

    def execute(self, userdata):
        try:
            goal = follow_meGoal()
            goal.start = userdata.request
            self.follow_me_act.send_goal(goal)
        except rospy.ServiceException, e:
            rospy.loginfo("[Service follow_me/client] call failed: %s", e)
        self.follow_me_act.wait_for_result()
        if userdata.request:
            rospy.loginfo('[Service follow_me] follow')
        else:
            rospy.loginfo('[Service follow_me] stop')
        return 'continue'

    def on_enter(self, userdata):
        self.follow_me_act.wait_for_server()

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

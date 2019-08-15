#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from std_msgs.msg import Bool

class hsr_Switcher(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># switch         bool

    <= true_load      repetition

    <= false_load     end

    '''

    def __init__(self):
        super(hsr_Switcher,self).__init__(outcomes=['true_load','false_load'],input_keys=['switch'])
        self.sw = None

    def execute(self, userdata):
        if self.sw:
            Logger.loginfo("Go to true_load")
            return 'true_load'
        else:
            Logger.loginfo("Go to false_load")
            return 'false_load'

    def on_enter(self, userdata):
        self.sw = userdata.switch

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

#!/usr/bin/env python
import rospy
from em_follow_me.srv import *
from flexbe_core import EventState, Logger

class hsr_try_limit(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    <= give_up         give_up
    <= try_again         try_again

    '''

    def __init__(self):
        super(hsr_try_limit,self).__init__(outcomes=['give_up', 'try_again'])
        self.counter = 0

    def execute(self, userdata):
        if self.counter > 1:
            self.counter = 0
            return 'give_up'
        else:
            return 'try_again'

    def on_enter(self, userdata):
        self.counter+=1

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

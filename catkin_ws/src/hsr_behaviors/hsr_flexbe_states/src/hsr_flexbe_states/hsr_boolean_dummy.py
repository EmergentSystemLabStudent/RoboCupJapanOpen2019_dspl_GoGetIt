#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger

class hsr_BooleanDummy(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- text          string     text

    #> follow_me          Bool     name

    <= continue      continue

    '''

    def __init__(self, text):
        super(hsr_BooleanDummy,self).__init__(outcomes=['continue'],output_keys=['follow_me'])

    def execute(self, userdata):
        userdata.follow_me = True
        return 'continue'

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

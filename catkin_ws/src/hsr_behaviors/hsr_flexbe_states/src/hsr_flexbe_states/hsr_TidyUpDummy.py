#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger


class hsr_TidyUpDummy(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    #> dummy1           string  dummy
    #> dummy2           string  dummy

    <= continue        continue
    '''

    def __init__(self):
        super(hsr_TidyUpDummy,self).__init__(outcomes=['continue'],output_keys=['dummy1', 'dummy2'])

    def execute(self, userdata):
        return 'continue'

    def on_enter(self, userdata):
        userdata.dummy1 = "dummy"
        userdata.dummy2 = "dummy"

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

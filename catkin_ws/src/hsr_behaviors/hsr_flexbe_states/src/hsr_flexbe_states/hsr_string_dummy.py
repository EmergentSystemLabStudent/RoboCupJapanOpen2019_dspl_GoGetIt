#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger

class hsr_StringDummy(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- text          string     text

    #> name          string     name

    <= continue      continue

    '''

    def __init__(self, text):
        super(hsr_StringDummy,self).__init__(outcomes=['continue'],output_keys=['name'])
        self._text = text

    def execute(self, userdata):
        userdata.name = self._text
        return 'continue'

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

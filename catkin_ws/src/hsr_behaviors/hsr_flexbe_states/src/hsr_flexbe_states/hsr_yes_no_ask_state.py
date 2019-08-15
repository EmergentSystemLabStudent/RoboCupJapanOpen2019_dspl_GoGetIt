#!/usr/bin/env python
import rospy
from em_follow_me.srv import *
from flexbe_core import EventState, Logger

class hsr_YesNoAsk(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># recognition    string  goal name

    <= yes     yes
    <= no      no
    '''

    def __init__(self):
        super(hsr_YesNoAsk,self).__init__(outcomes=['yes', 'no'],input_keys=['recognition'])
        self.text = str()

    def execute(self, userdata):
        if self.text == "Yes it is.":
            return 'yes'
        else:
            return 'no'

    def on_enter(self, userdata):
        self.text = userdata.recognition
        self.text = self.text.replace("-"," ")
        self.text = self.text.replace("  "," ")
        print self.text

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
import hsrb_interface
from hsrb_interface import geometry

class hsr_Move_Ahead(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- distance float distance
    <= continue        continue
    '''

    def __init__(self, distance = 0.0):
        super(hsr_Move_Ahead,self).__init__(outcomes=['continue'])
        self.robot      = hsrb_interface.Robot()
        self.omni_base   = self.robot.get('omni_base')
        self.rel_distance = distance

    def execute(self, userdata):
        self.omni_base.go_rel(0.0, self.rel_distance, 0.0, 100.0)
        return 'continue'

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

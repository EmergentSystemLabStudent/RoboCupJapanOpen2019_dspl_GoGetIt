#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
import hsrb_interface

class hsr_OmniBase(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- x            float
    -- y            float
    -- yaw          float
    -- time_out     float

    <= finish       finish
    '''

    def __init__(self, x=0.0, y=0.0, yaw=0.0, time_out=30.0):
        super(hsr_OmniBase,self).__init__(outcomes=['finish'])
        self.robot       = hsrb_interface.Robot()
        self.omni_base   = self.robot.get('omni_base')
        self._x          = x
        self._y          = y
        self._yaw        = yaw
        self._time_out   = time_out

    def execute(self, userdata):
        self.omni_base.go_rel(self._x, self._y, self._yaw, self._time_out)
        return 'finish'

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

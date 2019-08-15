#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
import moveit_commander
import hsrb_interface
from hsrb_interface import geometry

class hsr_MoveNeutral(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    <= continue        continue
    '''
    def __init__(self):
        super(hsr_MoveNeutral,self).__init__(outcomes=['continue'])
        self.robot      = hsrb_interface.Robot()
        self.whole_body = self.robot.get('whole_body')

    def execute(self, userdata):
            return 'continue'

    def on_enter(self, userdata):
        self.whole_body.move_to_neutral()

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

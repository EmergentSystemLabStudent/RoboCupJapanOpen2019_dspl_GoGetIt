#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

class hsr_DummyInitialPos(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    #> position     Pose    pose

    <= continue          continue
    '''

    def __init__(self):
        super(hsr_DummyInitialPos, self).__init__(outcomes=['continue'], output_keys=['position'])
        self.pos = Pose()

    def execute(self, userdata):
        userdata.position = self.pos
        return 'continue'

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

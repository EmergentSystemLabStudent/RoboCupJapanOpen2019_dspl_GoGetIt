#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
import hsrb_interface


class hsr_JointPoseLine(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.
    ># pose_goal           pose       Target configuration of the joints.
    -- line_z       float   line_z

    <= continue     Given time has passed.
    '''

    def __init__(self, line_z=0.0):
        super(hsr_JointPoseLine,self).__init__(outcomes=['continue'], input_keys=['pose_goal'])
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.get('whole_body')
        self._line_z = line_z

    def execute(self, userdata):
        return 'continue'

    def on_enter(self, userdata):
        try:
            self.whole_body.move_end_effector_by_line((1, 0, 0), max(min(userdata.pose_goal.position.z-0.7, 0.69), 0))
            self.whole_body.move_end_effector_by_line((0, 0, 1), self._line_z)
        except:
            pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
import hsrb_interface


class hsr_JointPose(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.
    -- arm_lift_joint       float   arm_lift_joint pose
    -- arm_flex_joint       float   arm_flex_joint pose
    -- arm_roll_joint       float   arm_roll_joint pose
    -- wrist_flex_joint     float   wrist_flex_joint pose
    -- wrist_roll_joint     float   wrist_roll_joint pose
    -- head_pan_joint       float   head_pan_joint pose
    -- head_tilt_joint      float   head_tilt_joint pose

    <= continue     Given time has passed.
    '''

    def __init__(self, arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0):
        super(hsr_JointPose,self).__init__(outcomes=['continue'])
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.get('whole_body')
        self._arm_lift_joint = arm_lift_joint
        self._arm_flex_joint = arm_flex_joint
        self._arm_roll_joint = arm_roll_joint
        self._wrist_flex_joint = wrist_flex_joint
        self._wrist_roll_joint = wrist_roll_joint
        self._head_pan_joint = head_pan_joint
        self._head_tilt_joint = head_tilt_joint

    def execute(self, userdata):
        return 'continue'

    def on_enter(self, userdata):
        try:
            self.whole_body.move_to_joint_positions({'arm_lift_joint': self._arm_lift_joint, 'arm_flex_joint': self._arm_flex_joint, 'arm_roll_joint': self._arm_roll_joint, 'wrist_flex_joint': self._wrist_flex_joint, 'wrist_roll_joint': self._wrist_roll_joint, 'head_pan_joint': self._head_pan_joint, 'head_tilt_joint': self._head_tilt_joint})
        except:
            pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

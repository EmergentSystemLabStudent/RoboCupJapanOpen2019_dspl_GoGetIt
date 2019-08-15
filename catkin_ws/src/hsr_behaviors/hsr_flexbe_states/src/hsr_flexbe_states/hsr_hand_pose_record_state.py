#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from flexbe_core import EventState, Logger
import hsrb_interface

class hsr_HandPoseRecord(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- ref_frame_id     string  referance frame id

    #> hand_pose        Pose    hand pose

    <= completed        finish
    '''

    def __init__(self, ref_frame_id):
        super(hsr_HandPoseRecord,self).__init__(outcomes=['completed'],output_keys=['hand_pose'])
        self.robot      = hsrb_interface.Robot()
        self.whole_body = self.robot.get('whole_body')
        self._ref_frame_id = ref_frame_id
        self.pose = Pose()

    def execute(self, userdata):
        userdata.hand_pose = self.pose
        return 'completed'

    def on_enter(self, userdata):
        arm = self.whole_body.get_end_effector_pose(ref_frame_id=self._ref_frame_id)
        self.pose.position.x = arm.pos.x
        self.pose.position.y = arm.pos.y
        self.pose.position.z = arm.pos.z
        self.pose.orientation.x = arm.ori.x
        self.pose.orientation.y = arm.ori.y
        self.pose.orientation.z = arm.ori.z
        self.pose.orientation.w = arm.ori.w

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

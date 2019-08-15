#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from flexbe_core import EventState, Logger

class hsr_MoveBaseClient(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.
    -- pose_position_x  Pose
    -- pose_position_y  Pose
    -- pose_orientation_z  Pose
    -- pose_orientation_w  Pose

    #> pose         Pose         test

    <= completed       finish
    '''

    def __init__(self, pose_position_x, pose_position_y, pose_orientation_z, pose_orientation_w):
        super(hsr_MoveBaseClient,self).__init__(outcomes=['completed'],output_keys=['pose'])
        self._pose = Pose()
        self._pose.position.x = pose_position_x
        self._pose.position.y = pose_position_y
        self._pose.orientation.z = pose_orientation_z
        self._pose.orientation.w = pose_orientation_w

    def execute(self, userdata):
        userdata.pose = self._pose
        return 'completed'

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

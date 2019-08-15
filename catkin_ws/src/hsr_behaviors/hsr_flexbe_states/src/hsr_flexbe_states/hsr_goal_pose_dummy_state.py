#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose

class hsr_GoalPoseDummy(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- px          float     px
    -- py          float     py
    -- pz          float     pz
    -- qx          float     qx
    -- qy          float     qy
    -- qz          float     qz
    -- qw          float     qw

    #> goal_pose     Pose     goal_pose

    <= continue      continue

    '''

    def __init__(self, px,py,pz,qx,qy,qz,qw):
        super(hsr_GoalPoseDummy,self).__init__(outcomes=['continue'],output_keys=['goal_pose'])
        self.px = px
        self.py = py
        self.pz = pz
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw

    def execute(self, userdata):
        goal_pose = Pose()
        goal_pose.position.x = self.px
        goal_pose.position.y = self.py
        goal_pose.position.z = self.pz
        goal_pose.orientation.x = self.qx
        goal_pose.orientation.y = self.qy
        goal_pose.orientation.z = self.qz
        goal_pose.orientation.w = self.qw
        userdata.goal_pose = goal_pose
        return 'continue'

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from flexbe_core import EventState, Logger
from hsrb_interface import Robot

class hsr_Move2Center(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- before          string    test

    -- after           string    test

    ># target_pose     dict      test

    <= continue             Given time has passed.
    <= failed                 Example for a failure outcome.

    '''

    def __init__(self, before='map', after='base_footprint'):
        super(hsr_Move2Center,self).__init__(outcomes=['continue','failed'],input_keys=['target_pose'])
        self._before = before
        self._after  = after
        self.pose_transformed = None
        robot = Robot()
        self.omni_base = robot.try_get('omni_base')

    def execute(self, userdata):
        return 'continue'

    def on_enter(self, userdata):
        self.pose_transformed = None
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        trans = tf_buffer.lookup_transform(
            self._after,
            self._before,
            rospy.Time(0),
            rospy.Duration(1.0))
        before_pose = userdata.target_pose
        pose = PoseStamped()
        pose.pose = before_pose
        transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
        self.pose_transformed = transformed.pose
        print self.pose_transformed
        try:
            self.omni_base.go_rel(0.0, self.pose_transformed.position.y, 0.0, 10.0)
        except:
            print("Time out")
    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

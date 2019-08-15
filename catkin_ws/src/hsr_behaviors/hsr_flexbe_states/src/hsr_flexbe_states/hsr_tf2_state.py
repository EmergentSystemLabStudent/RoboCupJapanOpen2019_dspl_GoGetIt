#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from flexbe_core import EventState, Logger


class hsr_Tf2(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- before          string    test

    -- after           string    test

    ># before_pose     dict      test

    #> after_pose      dict      test

    <= continue             Given time has passed.
    <= failed                 Example for a failure outcome.

    '''

    def __init__(self, before, after):
        super(hsr_Tf2,self).__init__(outcomes=['continue','failed'],input_keys=['before_pose'],output_keys=['after_pose'])
        self._before = before
        self._after  = after
        self.pose_transformed = {}

    def execute(self, userdata):
        userdata.after_pose = self.pose_transformed
        return 'continue'

    def on_enter(self, userdata):
        self.pose_transformed = {}
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        trans = tf_buffer.lookup_transform(
            self._after,
            self._before,
            rospy.Time(0),
            rospy.Duration(1.0))
        before_poses = [poses for poses in userdata.before_pose.items()]
        for i in range(len(before_poses)):
            pose = PoseStamped()
            pose.pose = before_poses[i][1]
            transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
            self.pose_transformed.update({before_poses[i][0]:transformed.pose})
        print self.pose_transformed

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

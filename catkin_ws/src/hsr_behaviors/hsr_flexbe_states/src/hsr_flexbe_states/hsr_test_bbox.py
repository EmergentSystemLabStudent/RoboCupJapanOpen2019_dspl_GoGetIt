#!/usr/bin/env python
import rospy
from octomap_msgs.srv import BoundingBoxQuery
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose

class hsr_TestBBox(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    #> object_point         Pose

    <= completed

    <= failed

    '''

    def __init__(self):
        super(hsr_TestBBox,self).__init__(outcomes=['completed','failed'],output_keys=['object_point'])
        self.bbox_pose = Pose()

    def execute(self, userdata):
        userdata.object_point = self.bbox_pose
        try:
            return 'completed'
        except:
            return 'failed'

    def on_enter(self, userdata):
        self.bbox_pose.position.x = 0.0
        self.bbox_pose.position.y = 0.0
        self.bbox_pose.position.z = 0.0

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Point
import hsrb_interface

class hsr_XtionPoint(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- ref_frame_id     string

    ># xtion_point      Point   xtion goal pose

    <= finish           finish
    '''

    def __init__(self, ref_frame_id):
        super(hsr_XtionPoint,self).__init__(outcomes=['finish'],input_keys=['xtion_point'])
        self.robot      = hsrb_interface.Robot()
        self.whole_body = self.robot.get('whole_body')
        self._ref_frame_id = ref_frame_id
        self.goal = Point()

    def execute(self, userdata):
        self.whole_body.gaze_point(point=hsrb_interface.geometry.Vector3(x=self.goal.x, y=self.goal.y, z=self.goal.z), ref_frame_id=self._ref_frame_id)
        self.whole_body.gaze_point(point=hsrb_interface.geometry.Vector3(x=self.goal.x, y=self.goal.y, z=self.goal.z), ref_frame_id=self._ref_frame_id)
        return 'finish'

    def on_enter(self, userdata):
        self.goal = userdata.xtion_point

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

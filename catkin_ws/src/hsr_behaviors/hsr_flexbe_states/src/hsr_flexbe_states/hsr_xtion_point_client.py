#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from flexbe_core import EventState, Logger

class hsr_XtionMoveClient(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- point_x  float
    -- point_y  float
    -- point_z  float

    #> point        Point   test

    <= completed    finish
    '''

    def __init__(self, point_x=0.0, point_y=0.0, point_z=0.0):
        super(hsr_XtionMoveClient,self).__init__(outcomes=['completed'],output_keys=['point'])
        self._point = Point()
        self._point.x = point_x
        self._point.y = point_y
        self._point.z = point_z

    def execute(self, userdata):
        userdata.point = self._point
        return 'completed'

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

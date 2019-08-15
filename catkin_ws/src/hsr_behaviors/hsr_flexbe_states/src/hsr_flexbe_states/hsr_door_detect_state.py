#!/usr/bin/env python
import rospy
from em_door_detect.srv import *
from flexbe_core import EventState, Logger

class hsr_DoorDetect(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    <= open            door is opening
    <= close           door is closing

    '''

    def __init__(self):
        super(hsr_DoorDetect,self).__init__(outcomes=['open','close'])
        self.em_door_detect_srv = rospy.ServiceProxy("em_door_detect", em_door_detect)

    def execute(self, userdata):
        try:
            res = self.em_door_detect_srv(True)
        except rospy.ServiceException, e:
            rospy.loginfo("[Service door_detect] call failed: %s", e)
        if res.result:
            rospy.loginfo("[Service door_detect] open")
            return 'open'
        else:
            rospy.loginfo("[Service door_detect] close")
            return 'close'

    def on_enter(self, userdata):
        rospy.wait_for_service("em_door_detect")

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

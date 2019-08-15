#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
from flexbe_core import EventState, Logger

class hsr_ResetOctomap(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- octomap_topic     string

    <= completed         reset the octomap

    <= failed            cannot reset the octomap

    '''

    def __init__(self, octomap_topic="/octomap_server/reset"):
        super(hsr_ResetOctomap,self).__init__(outcomes=['completed','failed'])
        self._topic = octomap_topic
        rospy.wait_for_service(self._topic)
        self.reset_octomap = rospy.ServiceProxy(self._topic, Empty)
    def execute(self, userdata):
        try:
            self.reset_octomap()
            return 'completed'
        except rospy.ServiceException, e:
            rospy.loginfo("[Service "+self._topic+"/client] call failed: %s", e)
            return 'failed'

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

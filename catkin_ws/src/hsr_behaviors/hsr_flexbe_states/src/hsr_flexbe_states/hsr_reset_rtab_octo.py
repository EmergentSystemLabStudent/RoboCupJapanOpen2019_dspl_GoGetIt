#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
from flexbe_core import EventState, Logger
#from std_msgs.msg import Bool

class hsr_ResetRtabmapandOctomap(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- octomap_topic     string

    -- play_octomap      Bool   Default: None

    <= completed         reset the octomap

    <= failed            cannot reset the octomap

    '''

    def __init__(self, octomap_topic="/octomap_server/reset", play_octomap=None):
        super(hsr_ResetRtabmapandOctomap,self).__init__(outcomes=['completed','failed'])
        self._topic = octomap_topic
        self.play_octomap = play_octomap

    def execute(self, userdata):
        self.reset_octomap = rospy.ServiceProxy(self._topic, Empty)
        self.reset_rtabmap = rospy.ServiceProxy('/reset', Empty)
        if self.play_octomap == False:
            self.pause_rtabmap = rospy.ServiceProxy('/pause', Empty)
        elif self.play_octomap == True:
            self.play_rtabmap = rospy.ServiceProxy('/resume', Empty)
        try:
            if self.play_octomap == True:
                self.play_rtabmap()
                Logger.loginfo("Reset and Play the Octomap")
            elif self.play_octomap == False:
                self.pause_rtabmap()
                Logger.loginfo("Reset and pause the Octomap")
            elif self.play_octomap == None:
                Logger.loginfo("Reset the Octomap")

            self.reset_rtabmap()
            self.reset_octomap()
            return 'completed'
        except rospy.ServiceException, e:
            rospy.loginfo("[Service "+self._topic+"/client] call failed: %s", e)
            return 'failed'

    def on_enter(self, userdata):
        rospy.wait_for_service(self._topic)

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

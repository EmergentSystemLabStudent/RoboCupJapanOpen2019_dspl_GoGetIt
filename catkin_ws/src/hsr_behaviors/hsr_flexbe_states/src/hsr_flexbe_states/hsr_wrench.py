#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import WrenchStamped

class hsr_GetWrench(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- threshold        float       StartBottun(20.0>0), PutControl(-15.0<0), Ignore(0.0)

    <= completed        Completed

    <= failed           Failed

    '''

    def __init__(self, threshold=-15.0):
        super(hsr_GetWrench,self).__init__(outcomes=['completed','failed'])
        self.threshold = threshold
        self._topic = '/hsrb/wrist_wrench/raw'
        self.sub_wrench = ProxySubscriberCached({self._topic: WrenchStamped})

    def execute(self, userdata):
        if self.sub_wrench.has_msg(self._topic):
            msg = self.sub_wrench.get_last_msg(self._topic)
            self.sub_wrench.remove_last_msg(self._topic)

            if self.threshold < 0.0 and msg.wrench.force.x < self.threshold:
                print "Move on Gripper Open Phase."
                return "completed"

            if self.threshold > 0.0 and msg.wrench.force.x > self.threshold:
                print "Start the Task."
                return "completed"

            if self.threshold == 0.0:
                print "Ignore the this state"
                return "completed"

        else:
            rospy.loginfo("Cannot subscribe the topic: %s", self._topic)
            return "failed"

    def on_enter(self, userdata):
        pass
        #self._sub_wrench = ProxySubscriberCached({self._topic: WrenchStamped})

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

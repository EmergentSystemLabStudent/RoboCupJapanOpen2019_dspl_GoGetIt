#!/usr/bin/env python
from flexbe_core import EventState, Logger

import rospy
from std_msgs.msg import String

class hsr_RosBridgeEvent(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- Event_ID     string  Event ID

    <= completed    completed

    <= failed       failed
    '''

    def __init__(self, Event_ID):
        super(hsr_RosBridgeEvent,self).__init__(outcomes=['completed','failed'])
        self.hsr_out = rospy.Publisher("/nrp_rosbridge_hsr_out", String, queue_size=1)
        self.event_id = String()
        self.event_id.data = Event_ID

    def execute(self, userdata):
        try:
            Logger.loginfo("Debug: "+ self.event_id.data)
            self.hsr_out.publish(self.event_id)
            return 'completed'
        except:
            Logger.loginfo("Missing the Publish Topic.")
            return 'failed'

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

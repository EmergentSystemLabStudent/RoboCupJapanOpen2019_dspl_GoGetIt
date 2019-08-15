#!/usr/bin/env python
import rospy, time
from flexbe_core import EventState, Logger
from std_msgs.msg import Bool

class hsr_PlanningSceneControl(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- update_sw          Bool   Default: True

    -- wait_time          float   Default: 0.0 [s]

    <= completed          clear the octomap

    <= failed             cannot clear the octomap

    '''

    def __init__(self, update_sw=True, wait_time=0.0):
        super(hsr_PlanningSceneControl,self).__init__(outcomes=['completed','failed'])
        self.sw = update_sw
        self.wait_time = wait_time
        self.pub_planning_scene_update = rospy.Publisher('/planning_scene/update', Bool, queue_size=1)
        self.msg = Bool()
    def execute(self, userdata):
        return 'completed'

    def on_enter(self, userdata):
        try:
            self.msg.data = self.sw
            if self.wait_time > 0.0:
                time.sleep(self.wait_time)
            self.pub_planning_scene_update.publish(self.msg)
            rospy.sleep(0.1)
            return 'completed'
        except:
            return 'failed'

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

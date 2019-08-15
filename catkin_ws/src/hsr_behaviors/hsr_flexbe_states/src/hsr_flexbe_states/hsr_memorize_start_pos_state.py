#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

class hsr_MemorizeStartPos(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    #> position     Pose    pose

    <= continue          continue
    '''

    def __init__(self):
        super(hsr_MemorizeStartPos, self).__init__(outcomes=['continue'], output_keys=['start_position'])
        rospy.Subscriber("global_pose", PoseStamped, self.callback)
        self.pos = Pose()

    def execute(self, userdata):
        userdata.start_position = self.pos
        return 'continue'

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def callback(self, data):
        #pos = PoseStamped()
        self.pos = data.pose
        #position = (pos.position.x,pos.position.y, pos.orientation.z, pos.orientation.w)

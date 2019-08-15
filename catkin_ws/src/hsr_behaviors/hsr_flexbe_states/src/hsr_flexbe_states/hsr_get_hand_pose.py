#!/usr/bin/env python
from flexbe_core import EventState, Logger

import rospy
import hsrb_interface
from geometry_msgs.msg import Pose

class hsr_GetHandPose(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    <= completed             Given time has passed.

    <= failed
    '''

    def __init__(self):
        super(hsr_GetHandPose,self).__init__(outcomes=['completed','failed'])
        self.pub_hand_pose = rospy.Publisher("/current/hand_pose", Pose, queue_size=1)
        self.hand_pose = Pose()

    def execute(self, userdata):
        try:
            self.hand_pose.position.x = self.arm.pos.x
            self.hand_pose.position.y = self.arm.pos.y
            self.hand_pose.position.z = self.arm.pos.z
            self.hand_pose.orientation.x = self.arm.ori.x
            self.hand_pose.orientation.y = self.arm.ori.y
            self.hand_pose.orientation.z = self.arm.ori.z
            self.hand_pose.orientation.w = self.arm.ori.w

            Logger.loginfo("Debug: "+str(self.hand_pose))
            self.pub_hand_pose.publish(self.hand_pose)
            rospy.sleep(0.3)
            return 'completed'
        except:
            Logger.loginfo("Miss the Getting Hand Pose.")
            return 'failed'

    def on_enter(self, userdata):
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.get('whole_body')
        self.arm = self.whole_body.get_end_effector_pose(ref_frame_id="map")
        Logger.loginfo("Get the HSR's Hand Pose")

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

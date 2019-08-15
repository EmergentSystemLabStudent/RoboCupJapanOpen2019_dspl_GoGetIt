#!/usr/bin/env python
import rospy
import sys
from flexbe_core import EventState, Logger
import hsrb_interface
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped, Point
import math
from hsrb_interface import geometry


class hsr_GrippingObject(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- grasp_force          float   grasp force (default: 0.2)

    -- mode             bool   grasp or open. grasp:True, open:False. (default: True)

    #> grasp_success str success:True, failed: False

    <= continue             transition leading to completion

    <= failed                 transition leading to completion failure
    '''

    def __init__(self, grasp_force=0.7, mode=True):
        super(hsr_GrippingObject,self).__init__(outcomes=['continue','failed'],output_keys=['grasp_success'])
        self._grasp_force = grasp_force
        self.grasp_success = False
        self._mode = mode
        self.robot = hsrb_interface.Robot()
        self.omni_base = self.robot.get('omni_base')
        self.whole_body = self.robot.get('whole_body')
        self.gripper = self.robot.get('gripper')
        self.tts = self.robot.get('default_tts')


    def execute(self, userdata):
        if self.grasp_success is False:
            return 'failed'
        else:
            return 'continue'

    def on_enter(self, userdata):
        if self._mode is True:
            try:
                self.gripper.apply_force(self._grasp_force)
                tf_buffer = tf2_ros.Buffer()
                tf_listener = tf2_ros.TransformListener(tf_buffer)
                after = 'hand_l_finger_tip_frame'
                before = 'hand_r_finger_tip_frame'
                function = tf_buffer.lookup_transform(after, before, rospy.Time(0), rospy.Duration(1.0))
                fin_distance = math.sqrt((function.transform.translation.x**2) + (function.transform.translation.y**2) + (function.transform.translation.z**2))
                self.whole_body.move_to_joint_positions({'arm_lift_joint':self.whole_body.joint_positions['arm_lift_joint']+0.05})
                print("fin_distance: ", fin_distance)
                if fin_distance > 0.008:
                    self.grasp_success = True
                    print(True)
                else:
                    self.grasp_success = False
                    userdata.grasp_success = 'not_grasp'
                    print(False)
            except:
                print('Faild gripping!')
                userdata.grasp_success = 'not_grasp'
                rospy.logerr('fail to grasp')
        else:
            try:
                self.gripper.command(1.2)
                self.grasp_success = True
            except:
                print('Faild hand Open!')


    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

#!/usr/bin/env python
import rospy
import copy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import moveit_commander
import math
import numpy as np
from flexbe_core import EventState, Logger
import actionlib
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveItErrorCodes
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from tf.transformations import quaternion_from_euler, quaternion_about_axis, quaternion_multiply
from moveit_util import *
from import_tidy_up_stage2 import *
import hsrb_interface
from hsrb_interface import geometry


class hsr_CreateApproach(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- offset    float   Default: None

    ># target_pose     pose      target_pose

    #> selected_pose_approach     pose      selected_pose_approach

    #> selected_pose_grasp     pose      selected_pose_grasp

    <= continue             Given time has passed.

    <= failed                 Example for a failure outcome.

    '''

    def __init__(self, offset=0.3):
        super(hsr_CreateApproach,self).__init__(outcomes=['continue','failed'],input_keys=['target_pose'], output_keys=['selected_pose_approach', 'selected_pose_grasp'])
        self._offset = offset

        self.selected_pose_approach = None
        self.selected_pose_grasp = None
        self._move_group = 'whole_body'
        group = moveit_commander.MoveGroupCommander(self._move_group)

        self.robot = hsrb_interface.Robot()
        self.current_pose = None


    def execute(self, userdata):
        userdata.selected_pose_approach = self.selected_pose_approach
        userdata.selected_pose_grasp = self.selected_pose_grasp
        if self.selected_pose_approach is None:
            return 'failed'
        else:
            return 'continue'

    def on_enter(self, userdata):
        # Update self.current_pose.
        whole_body = self.robot.get('whole_body')
        self.current_pose = whole_body.get_end_effector_pose(ref_frame_id='map')
        target_pose = userdata.target_pose
        self.selected_pose_grasp = target_pose
        print("target: ", target_pose)
        if target_pose != None:
            print('target_pose:', target_pose)
            distance = \
            math.sqrt(pow((target_pose.position.x - self.current_pose.pos.x), 2) \
            + pow((target_pose.position.y - self.current_pose.pos.y), 2) \
            + pow((target_pose.position.z - self.current_pose.pos.z), 2))
            self.selected_pose_approach = copy.deepcopy(target_pose)
            fix_orientation =self.quaternion_from_position(self.current_pose.pos, target_pose.position)
            self.selected_pose_grasp.orientation.x, self.selected_pose_grasp.orientation.y, \
            self.selected_pose_grasp.orientation.z, self.selected_pose_grasp.orientation.w = fix_orientation
            self.selected_pose_approach.position.x = target_pose.position.x \
            - self._offset / distance * (target_pose.position.x - self.current_pose.pos.x)
            self.selected_pose_approach.position.y = target_pose.position.y \
            - self._offset / distance * (target_pose.position.y - self.current_pose.pos.y)
            self.selected_pose_approach.orientation = self.selected_pose_grasp.orientation
            print("approach: ", self.selected_pose_approach)
            # if self.check_plan(pose = self.selected_pose_grasp, orientation=True) == False:
            #     self.selected_pose_approach = None
            #     self.selected_pose_grasp = None
            #     self.selected_object_status = 'fail_plan'
        else:
            Logger.logwarn('detect result is empty')
            self.selected_pose_approach = None


    def on_exit(self, userdata):
        # Clear self.current_pose.
        self.current_pose = None
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def quaternion_from_position(self, pos1, pos2):
        x = math.atan2((pos2.z - pos1.z), (pos2.y - pos1.y))
        #x = math.atan2(0, (pos2.y - pos1.y))
        #x = 0

        y = math.atan2((pos2.x - pos1.x), (pos2.z - pos1.z))
        #y = 0

        z = math.atan2((pos2.y - pos1.y), (pos2.x - pos1.x))
        #z = 1.57
        orig = quaternion_from_euler(z, y, x)
        orig = quaternion_from_euler(-1.57, z+3.14, 0, 'syzy')
        #rotationX90 = quaternion_about_axis(math.pi/2 ,[1, 0, 0])
        #rotationY90 = quaternion_about_axis(math.pi/2 ,[0, 1, 0])
        #rotationZ90 = quaternion_about_axis(math.pi/2, [0, 0, 1])
        #orig = quaternion_multiply(orig, rotationY90)
        #orig = quaternion_multiply(orig, rotationZ90)
        #orig = quaternion_multiply(orig, rotationX90)
        #orig = quaternion_multiply(orig, rotationX90)
        #orig = quaternion_multiply(orig, rotationX90)
        #orig = quaternion_multiply(orig, rotationY90)
        #orig = quaternion_multiply(orig, rotationX90)
        #orig = quaternion_multiply(orig, rotationX90)
        #orig = quaternion_multiply(orig, rotationX90)

        return orig

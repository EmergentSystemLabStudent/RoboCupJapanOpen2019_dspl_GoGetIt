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


class hsr_PoseDecision(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># target_poses     dict      target_poses

    ># category     string      category

    ># failed_objects dict       failed_objects

    #> selected_pose_grasp    pose      selected_pose_grasp

    #> selected_object_name    string      selected_object_name

    #> selected_object_status    string      selected_object_status

    <= continue             Given time has passed.

    <= failed                 Example for a failure outcome.

    '''

    def __init__(self):
        super(hsr_PoseDecision,self).__init__(outcomes=['continue','failed'],input_keys=['target_poses', 'category', 'failed_objects'], output_keys=['selected_pose_grasp', 'selected_object_name', 'selected_object_status'])
        self.known_dict = known_dict
        self.unknown_list = unknown_list

        self.selected_object_name = None
        self.selected_object_status = 'not_detect'
        self.selected_pose_grasp = None
        self._move_group = 'whole_body'
        group = moveit_commander.MoveGroupCommander(self._move_group)
        #self.current_pose = group.get_current_pose().pose
        #print(self.current_pose)
        self.eef_link = group.get_end_effector_link()
        self.planning_frame = group.get_planning_frame()
        self.planning_frame = '/map'
        self._tolerance = 0.01

        self.robot = hsrb_interface.Robot()
        self.current_pose = None

        self._action_topic = '/move_group'
        #self._client = ProxyActionClient({self._action_topic: MoveGroupAction})
        self._client = actionlib.SimpleActionClient(self._action_topic, MoveGroupAction)
        self._client.wait_for_server()


    def execute(self, userdata):
        userdata.selected_pose_grasp = self.selected_pose_grasp
        userdata.selected_object_name = self.selected_object_name
        userdata.selected_object_status = self.selected_object_status
        if self.selected_pose_grasp is None:
            return 'failed'
        else:
            return 'continue'

    def on_enter(self, userdata):
        # Update self.current_pose.
        whole_body = self.robot.get('whole_body')
        self.current_pose = whole_body.get_end_effector_pose(ref_frame_id='map')
        target_poses = [poses for poses in userdata.target_poses.items()]
        #target_poses = userdata.target_poses
        category = userdata.category
        # failed_objects = userdata.failed_objects
        # collections = {"known": self.known_dict.keys(), "unknown": self.unknown_list}
        candidates = [category]
        print("category: ", candidates)
        target_pose = [pose for pose in target_poses if pose[0] in candidates]
        print("target: ", target_pose)
        # target_pose = [pose for pose in target_pose if not pose[0] in [object for object, value in failed_objects.items() if value is True]]
        if target_pose != []:
            print('target_pose:', target_pose)
            distance_array = np.array([])
            for pose in target_pose:
                distance = \
                math.sqrt(pow((pose[1].position.x - self.current_pose.pos.x), 2) \
                + pow((pose[1].position.y - self.current_pose.pos.y), 2) \
                + pow((pose[1].position.z - self.current_pose.pos.z), 2))
                distance_array = np.append(distance_array, distance)
            distance_index = np.argsort(distance_array)
            self.selected_pose_grasp = copy.deepcopy(target_pose[distance_index[0]][1])
            self.selected_object_name = target_pose[distance_index[0]][0]
            self.selected_object_status = 'Success!'


            fix_orientation =self.quaternion_from_position(self.current_pose.pos, self.selected_pose_grasp.position)
            self.selected_pose_grasp.orientation.x, self.selected_pose_grasp.orientation.y, \
            self.selected_pose_grasp.orientation.z, self.selected_pose_grasp.orientation.w = fix_orientation
            print("grasp: ", self.selected_pose_grasp)
            print("name: ", self.selected_object_name)
            # if self.check_plan(pose = self.selected_pose_grasp, orientation=True) == False:
            #     self.selected_pose_approach = None
            #     self.selected_pose_grasp = None
            #     self.selected_object_status = 'fail_plan'
            print('object_name: ', self.selected_object_name)
            print('object_status: ', self.selected_object_status)
        else:
            Logger.logwarn('detect result is empty')
            self.selected_pose_grasp = None
            self.selected_object_name = None
            self.selected_object_status = 'not_detect'


    def on_exit(self, userdata):
        # Clear self.current_pose.
        self.current_pose = None
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def check_plan(self, pose, orientation=False):
        action_goal = make_default_action_goal(self, pose, orientation)
        action_goal.planning_options.planning_scene_diff.is_diff = True
        action_goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        action_goal.planning_options.plan_only = True
        try:
            self._client.send_goal(action_goal)
            self._client.wait_for_result()
            res = self._client.get_result()
            print(res.error_code.val, self.planning_frame)
            if res.error_code.val < 1:
                return False
            else:
                return True

        except Exception as e:
            Logger.logwarn('Failed to send action goal for group: %s\n%s' % (self._move_group, str(e)))

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

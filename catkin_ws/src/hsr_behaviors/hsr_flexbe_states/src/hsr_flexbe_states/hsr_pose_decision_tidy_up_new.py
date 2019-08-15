#!/usr/bin/env python
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose, PoseStamped
import math, moveit_commander, actionlib
import numpy as np
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveItErrorCodes
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from tf.transformations import quaternion_from_euler, quaternion_about_axis, quaternion_multiply
from moveit_util_new import *
import hsrb_interface
from hsrb_interface import geometry


class hsr_PoseDecisionTidyUpNew(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- offset                     float      Default: None

    -- tolerance                  float      Default: 0.001

    -- multi_plan                 bool       Default: False

    -- multi_plan_num             int        Default: 3

    -- orien_tolerance            bool       Default: True

    ># target_poses               dict       target_poses

    ># target_object_names        list       object_names

    #> selected_pose_approach     pose       selected_pose_approach

    #> selected_pose_grasp        pose       selected_pose_grasp

    #> selected_object_name       string     selected_object_name

    #> selected_object_status     string     selected_object_status

    #> plan_num                   int        plan_num

    <= continue                   Given time has passed.

    <= plan_failed                Example for a failure outcome.

    <= detect_failed              Example for a failure outcome.

    '''

    def __init__(self, offset=None, tolerance=0.001, multi_plan=False, multi_plan_num=3, orien_tolerance=True):
        super(hsr_PoseDecisionTidyUpNew, self).__init__(outcomes=['continue', 'plan_failed', 'detect_failed'], input_keys=['target_poses', 'target_object_names'], output_keys=['selected_pose_approach', 'selected_pose_grasp', 'selected_object_name', 'selected_object_status','plan_num'])
        self._offset = offset
        self._tolerance = tolerance
        self._multi_plan = multi_plan
        self._multi_plan_num = multi_plan_num
        self.orien_tolerance = orien_tolerance
        self.count_num = 0

        self.selected_object_name = None
        self.selected_object_status = 'not_detect'
        self.selected_pose_grasp = None
        self.selected_pose_approach = None
        self._move_group = 'whole_body'
        group = moveit_commander.MoveGroupCommander(self._move_group)

        self.eef_link = group.get_end_effector_link()
        self.planning_frame = group.get_planning_frame()
        self.planning_frame = '/map'

        self.robot = hsrb_interface.Robot()
        self.current_pose = None

        self._action_topic = '/move_group'
        self._client = actionlib.SimpleActionClient(self._action_topic, MoveGroupAction)
        self._client.wait_for_server()

    def execute(self, userdata):
        userdata.selected_pose_approach = self.selected_pose_approach
        userdata.selected_pose_grasp = self.selected_pose_grasp
        userdata.selected_object_name = self.selected_object_name
        userdata.selected_object_status = self.selected_object_status
        if self._multi_plan == True:
            userdata.plan_num = self.count_num
        else:
            userdata.plan_num = None
        if self.selected_pose_grasp is None and self.selected_pose_approach is None:
            if self.selected_object_status == 'not_detect':
                return 'detect_failed'
            elif self.selected_object_status == 'fail_plan':
                return 'plan_failed'
        else:
            return 'continue'

    def on_enter(self, userdata):
        self.count_num = None
        # Update self.current_pose.
        whole_body = self.robot.get('whole_body')
        self.current_pose = whole_body.get_end_effector_pose(ref_frame_id='map')
        target_poses = [poses for poses in userdata.target_poses.items()]
        target_object_names = userdata.target_object_names
        if len(target_poses) == 0:
            Logger.logwarn('detect result is empty')
            self.selected_pose_approach = None
            self.selected_pose_grasp = None
            self.selected_object_name = None
            self.selected_object_status = 'not_detect'
        else:
            #failed_objects = userdata.failed_objects
            # target_pose = [pose for pose in [target_pose for target_pose in target_poses if target_pose[0]
            #                                  in target_object_names] if not pose[0] in [object for object, value in failed_objects.items() if value is True]]
            target_pose = [pose for pose in [target_pose for target_pose in target_poses if target_pose[0] in target_object_names]]
            print('detect_object:', target_poses)
            print('target_pose:', target_pose)
            print('target_name:', target_object_names)
            if target_pose != []:
                distance_array = np.array([]) # change HERE for my work.
                for pose in target_pose:
                    distance = \
                        math.sqrt(pow((pose[1].position.x - self.current_pose.pos.x), 2) + pow((pose[1].position.y - self.current_pose.pos.y), 2) + pow((pose[1].position.z - self.current_pose.pos.z), 2))
                    distance_array = np.append(distance_array, distance)
                distance_index = np.argsort(distance_array)
                self.selected_pose_grasp = target_pose[distance_index[0]][1]
                self.selected_pose_approach = target_pose[distance_index[0]][1]
                self.selected_object_name = target_pose[distance_index[0]][0]
                self.selected_object_status = 'Success!'
                if self._offset is not None:
                    print("Valid offset")
                    self.selected_pose_approach.position.x = self.selected_pose_grasp.position.x - self._offset / distance_array[0] * (self.selected_pose_grasp.position.x - self.selected_pose_grasp.position.x)
                    self.selected_pose_approach.position.y = self.selected_pose_grasp.position.y - self._offset / distance_array[0] * (self.selected_pose_grasp.position.y - self.selected_pose_grasp.position.y)
                    # self.selected_pose_approach.position.z = self.selected_pose_grasp.position.z + self._offset

                if self._multi_plan == True:
                    for p_n in xrange(self._multi_plan_num):
                        self.count_num = p_n
                        fix_orientation = self.fix_orientation(self.selected_pose_grasp)
                        if fix_orientation is not None:
                            self.selected_pose_grasp.orientation.x, self.selected_pose_grasp.orientation.y, \
                                self.selected_pose_grasp.orientation.z, self.selected_pose_grasp.orientation.w = fix_orientation
                            self.selected_pose_approach.orientation = self.selected_pose_grasp.orientation
                            print(self.selected_pose_grasp)
                            print(self.selected_object_name)
                            if self.fix_orientation(pose=self.selected_pose_grasp, orientation=self.orien_tolerance, plan_sw=False) is None and self.count_num == self._multi_plan_num-1:
                                self.selected_pose_approach = None
                                self.selected_pose_grasp = None
                                self.selected_object_status = 'fail_plan'
                            else:
                                print('object_name: ', self.selected_object_name)
                                print('object_status: ', self.selected_object_status)
                                break
                        elif self.count_num == self._multi_plan_num-1:
                            self.selected_pose_approach = None
                            self.selected_pose_grasp = None
                            self.selected_object_status = 'fail_plan'
                            print("Failed plan!")
                else:
                    fix_orientation = self.fix_orientation(self.selected_pose_grasp)
                    if fix_orientation is not None:
                        self.selected_pose_grasp.orientation.x, self.selected_pose_grasp.orientation.y, \
                            self.selected_pose_grasp.orientation.z, self.selected_pose_grasp.orientation.w = fix_orientation
                        self.selected_pose_approach.orientation = self.selected_pose_grasp.orientation
                        print(self.selected_pose_grasp)
                        print(self.selected_object_name)
                        if self.fix_orientation(pose=self.selected_pose_approach, orientation=self.orien_tolerance, plan_sw=False) is None:
                            self.selected_pose_approach = None
                            self.selected_pose_grasp = None
                            self.selected_object_status = 'fail_plan'
                        else:
                            print('object_name: ', self.selected_object_name)
                            print('object_status: ', self.selected_object_status)
                    else:
                        self.selected_pose_approach = None
                        self.selected_pose_grasp = None
                        self.selected_object_status = 'fail_plan'
                        print("Failed plan!")
            else:
                Logger.logwarn('detect result is empty')
                self.selected_pose_approach = None
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

    def fix_orientation(self, pose, orientation=False, plan_sw=True):
        action_goal = make_default_action_goal(self, pose, orientation)
        action_goal.planning_options.planning_scene_diff.is_diff = True
        action_goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        action_goal.planning_options.plan_only = plan_sw
        try:
            self._client.send_goal(action_goal)
            self._client.wait_for_result()
            res = self._client.get_result()
            print(res.error_code.val, self.planning_frame)
            if res.error_code.val < 1:
                return None
            else:
                return self.quaternion_from_position(self.current_pose.pos, self.selected_pose_grasp.position)

        except Exception as e:
            Logger.logwarn('Failed to send action goal for group: %s\n%s' % (
                self._move_group, str(e)))

    def quaternion_from_position(self, pos1, pos2):
        x = math.atan2((pos2.z - pos1.z), (pos2.y - pos1.y))

        y = math.atan2((pos2.x - pos1.x), (pos2.z - pos1.z))

        z = math.atan2((pos2.y - pos1.y), (pos2.x - pos1.x))

        orig = quaternion_from_euler(z, y, x)
        orig = quaternion_from_euler(-1.57, z+3.14, 0, 'syzy')

        return orig

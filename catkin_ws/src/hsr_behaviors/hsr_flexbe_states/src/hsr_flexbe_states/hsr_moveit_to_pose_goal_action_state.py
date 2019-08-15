#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveItErrorCodes
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
import moveit_commander
from moveit_util import *
'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class hsr_MoveitToPoseGoalAction(EventState):
    '''
    Uses MoveIt to plan and move the specified joints to the target configuration.

    -- move_group          string     Name of the move group to be used for planning.Specified joint names need to exist in the given group.

    -- action_topic        string     Topic on which MoveIt is listening for action calls.

    -- tolerance           float      Default: 0.001

    -- orien_tolerance     bool       Default: True

    ># pose_goal           pose       Target configuration of the joints.

    #> move_status         string     move_status

    <= reached             Target joint configuration has been reached.

    <= planning_failed     Failed to find a plan to the given joint configuration.

    <= control_failed      Failed to move the arm along the planned trajectory.

    '''


    def __init__(self, move_group='whole_body', action_topic = '/move_group', tolerance=0.001, orien_tolerance=True):
        '''
        Constructor
        '''
        super(hsr_MoveitToPoseGoalAction, self).__init__(outcomes=['reached', 'planning_failed', 'control_failed'], output_keys=['move_status'], input_keys=['pose_goal'])

        self._action_topic = action_topic
        self._client = ProxyActionClient({self._action_topic: MoveGroupAction})
        self._tolerance = tolerance
        self.orien_tolerance = orien_tolerance

        self._move_group = move_group
        self._planning_failed = False
        self._control_failed = False
        self._success = False

        robot = moveit_commander.RobotCommander()
        group = moveit_commander.MoveGroupCommander(self._move_group)
        self.planning_frame = group.get_planning_frame()
        self.planning_frame = '/map'
        self.eef_link = group.get_end_effector_link()
        group_names = robot.get_group_names()
        scene = moveit_commander.PlanningSceneInterface()


    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self._planning_failed:
            userdata.move_status = 'fail_plan'
            return 'planning_failed'
        if self._control_failed:
            return 'control_failed'
        if self._success:
            return 'reached'

        if self._client.has_result(self._action_topic):
            result = self._client.get_result(self._action_topic)

            if result.error_code.val == MoveItErrorCodes.CONTROL_FAILED:
                Logger.logwarn('Control failed for move action of group: %s (error code: %s)' % (self._move_group, str(result.error_code)))
                self._control_failed = True
                return 'control_failed'
            elif result.error_code.val != MoveItErrorCodes.SUCCESS:
                Logger.logwarn('Move action failed with result error code: %s' % str(result.error_code))
                self._planning_failed = True
                return 'planning_failed'
            else:
                print("[on_execute]userdata.pose_goal=", userdata.pose_goal)
                self._success = True
                return 'reached'


    def on_enter(self, userdata):
        print("[on_enter]userdata.pose_goal=", userdata.pose_goal)
        self._planning_failed = False
        self._control_failed = False
        self._success = False

        print('planning_frame:', self.planning_frame)
        print('eef_link:', self.eef_link)
        exec_pose = userdata.pose_goal

        action_goal = make_default_action_goal(self, exec_pose, orientation=self.orien_tolerance)
        action_goal.planning_options.planning_scene_diff.is_diff = True
        action_goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        action_goal.planning_options.plan_only = False
        print("[on_enter]action_goal=", action_goal)
        try:
            self._client.send_goal(self._action_topic, action_goal)
        except Exception as e:
            Logger.logwarn('Failed to send action goal for group: %s\n%s' % (self._move_group, str(e)))
            self._planning_failed = True


    def on_stop(self):
        try:
            if self._client.is_available(self._action_topic) \
            and not self._client.has_result(self._action_topic):
                self._client.cancel(self._action_topic)
        except:
            # client already closed
            pass

    def on_pause(self):
        self._client.cancel(self._action_topic)

    def on_resume(self, userdata):
        self.on_enter(userdata)

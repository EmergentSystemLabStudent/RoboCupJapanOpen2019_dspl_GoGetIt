import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import moveit_commander
import math
import actionlib
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveItErrorCodes
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

def make_default_action_goal(object, pose, orientation=True):
    action_goal = MoveGroupGoal()
    action_goal.request.group_name = object._move_group
    action_goal.request.start_state.is_diff = True
    action_goal.request.planner_id = "PRMkConfigDefault"

    goal_constraint = Constraints()

    goal_constraint.position_constraints.append(PositionConstraint())
    goal_constraint.position_constraints[0].header.frame_id = object.planning_frame
    goal_constraint.position_constraints[0].link_name = object.eef_link
    bounding_volume = BoundingVolume()
    solid_primitive = SolidPrimitive()
    solid_primitive.dimensions = [object._tolerance * object._tolerance]
    solid_primitive.type = solid_primitive.SPHERE
    bounding_volume.primitives.append(solid_primitive)
    bounding_volume.primitive_poses.append(pose)
    goal_constraint.position_constraints[0].constraint_region = bounding_volume
    goal_constraint.position_constraints[0].weight = 1.0
    if orientation is not False:
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = object.planning_frame
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.link_name = object.eef_link
        orientation_constraint.absolute_x_axis_tolerance = object._tolerance
        orientation_constraint.absolute_y_axis_tolerance = object._tolerance
        orientation_constraint.absolute_z_axis_tolerance = object._tolerance
        orientation_constraint.weight = 1.0
        goal_constraint.orientation_constraints.append(orientation_constraint)
    action_goal.request.goal_constraints.append(goal_constraint)
    action_goal.request.num_planning_attempts = 5
    action_goal.request.allowed_planning_time = 5
    action_goal.request.workspace_parameters.header.frame_id = object.planning_frame
    action_goal.request.workspace_parameters.min_corner.x = -20
    action_goal.request.workspace_parameters.min_corner.y = -20
    action_goal.request.workspace_parameters.min_corner.z = -20
    action_goal.request.workspace_parameters.max_corner.x = 20
    action_goal.request.workspace_parameters.max_corner.y = 20
    action_goal.request.workspace_parameters.min_corner.z = 20
    action_goal.planning_options.look_around = False
    action_goal.planning_options.replan = True
    return action_goal

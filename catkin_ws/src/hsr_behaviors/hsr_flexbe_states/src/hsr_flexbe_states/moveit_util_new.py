### coding: UTF-8
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
    action_goal.request.planner_id = "RRTkConfigDefault"# PRMkConfigDefault, PRMstarkConfigDefault, RRTkConfigDefault, RRTstarkConfigDefault

    goal_constraint = Constraints()

    goal_constraint.position_constraints.append(PositionConstraint())
    goal_constraint.position_constraints[0].header.frame_id = object.planning_frame
    goal_constraint.position_constraints[0].link_name = object.eef_link
    bounding_volume = BoundingVolume()
    solid_primitive = SolidPrimitive()
    solid_primitive.dimensions = [object._tolerance]
    solid_primitive.type = solid_primitive.SPHERE
    bounding_volume.primitives.append(solid_primitive)

    # if object.selected_object_name[:8] == "postcard":
    #     p_update = pose
    #     p_update.position.z = 0.75
    #     bounding_volume.primitive_poses.append(p_update)
    #elif pose.position.z <= 0.1: #地面に当たらないように
    # if pose.position.z <= 0.1: #地面に当たらないように
    #     p_update = pose
    #     p_update.position.z = 0.13
    #     bounding_volume.primitive_poses.append(p_update)
    # else:
    bounding_volume.primitive_poses.append(pose)

    goal_constraint.position_constraints[0].constraint_region = bounding_volume
    goal_constraint.position_constraints[0].weight = 1.0

    if orientation is not False:
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = object.planning_frame
        if pose.position.z <= 0.2: # Stage 2 (Grasp)
            rospy.loginfo("Planning num:0-0 (Grasp)")
            orientation_constraint.orientation.x = 0.675
            orientation_constraint.orientation.y = 0.737
            orientation_constraint.orientation.z = 0.018
            orientation_constraint.orientation.w = -0.019
            #goal_constraint.position_constraints[0].constraint_region.primitive_poses[0].position.z -= 0.01 # Stage1

        # elif object.selected_object_name[:8] == "postcard":
        # #elif pose.position.z < 0.02: # Stage 1
        #     object.
        #     orientation_constraint.orientation.x = 0.0#0.904
        #     orientation_constraint.orientation.y = 0.0#-0.166
        #     orientation_constraint.orientation.z = -0.676#-0.068
        #     orientation_constraint.orientation.w = 0.736#0.387
        #     #orientation_constraint.link_name = "hand_palm_link"
        elif object.count_num == 1: # Stage 2 (Sofa)
            rospy.loginfo("Planning num:1 (Put on sofa)")
            orientation_constraint.orientation.x = -0.019
            orientation_constraint.orientation.y = -0.707
            orientation_constraint.orientation.z = -0.202
            orientation_constraint.orientation.w = 0.706
        elif object.count_num == 2: # Stage 2 (Work desk, Shelf)
            rospy.loginfo("Planning num:2 (Put on shelf or desk)")
            orientation_constraint.orientation.x = 0.51
            orientation_constraint.orientation.y = -0.489
            orientation_constraint.orientation.z = 0.509
            orientation_constraint.orientation.w = 0.489
        elif object.count_num == 3: # Stage 2 (White table)
            rospy.loginfo("Planning num:3 (Put on table)")
            orientation_constraint.orientation.x = 0.51
            orientation_constraint.orientation.y = 0.489
            orientation_constraint.orientation.z = 0.501
            orientation_constraint.orientation.w = -0.497
        else:
            rospy.loginfo("Planning num:0-1 (Put)")
            orientation_constraint.orientation = pose.orientation
        orientation_constraint.link_name = object.eef_link
        orientation_constraint.absolute_x_axis_tolerance = object._tolerance*100
        orientation_constraint.absolute_y_axis_tolerance = object._tolerance*100
        orientation_constraint.absolute_z_axis_tolerance = object._tolerance*100
        orientation_constraint.weight = 1.0
        goal_constraint.orientation_constraints.append(orientation_constraint)

    action_goal.request.goal_constraints.append(goal_constraint)
    action_goal.request.num_planning_attempts = 5
    action_goal.request.allowed_planning_time = 10.0
    action_goal.request.workspace_parameters.header.frame_id = object.planning_frame
    action_goal.request.workspace_parameters.min_corner.y = -5.0#-20.0
    action_goal.request.workspace_parameters.min_corner.x = -5.0#-20.0
    action_goal.request.workspace_parameters.min_corner.z = -5.0#-20.0
    action_goal.request.workspace_parameters.max_corner.x = 5.0#20.0
    action_goal.request.workspace_parameters.max_corner.y = 5.0#20.0
    action_goal.request.workspace_parameters.min_corner.z = 5.0#20.0
    action_goal.planning_options.look_around = False
    action_goal.planning_options.replan = True
    action_goal.planning_options.replan_attempts = 5
    action_goal.planning_options.replan_delay = 10.0
    return action_goal

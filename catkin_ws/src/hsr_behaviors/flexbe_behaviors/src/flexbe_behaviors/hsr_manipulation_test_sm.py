#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_joint_pose_state import hsr_JointPose
from hsr_flexbe_states.hsr_bounding_box_2d_state import hsr_BoundingBox2D
from hsr_flexbe_states.hsr_tf2_state import hsr_Tf2
from hsr_flexbe_states.hsr_pose_decision import hsr_PoseDecision
from hsr_flexbe_states.hsr_moveit_to_pose_goal_action_state import hsr_MoveitToPoseGoalAction
from hsr_flexbe_states.hsr_grasping_point_detect_state import hsr_GraspingPointDetect
from hsr_flexbe_states.hsr_pose_dummy import hsr_PoseDummy
from hsr_flexbe_states.hsr_image_capture_state import hsr_ImageCapture
from hsr_flexbe_states.hsr_veiw_marker import hsr_ViewMarker
from hsr_flexbe_states.hsr_gripping_object_state import hsr_GrippingObject
from hsr_flexbe_states.hsr_collision_box_state import hsr_CollisionBox
from hsr_flexbe_states.hsr_move_to_neutral import hsr_MoveNeutral
from hsr_flexbe_states.hsr_joint_pose_line_state import hsr_JointPoseLine
from hsr_flexbe_states.hsr_move2center import hsr_Move2Center
from hsr_flexbe_states.hsr_create_approach import hsr_CreateApproach
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 21 2019
@author: tfukui
'''
class hsr_manipulation_testSM(Behavior):
	'''
	hsr_manipulation_test
	'''


	def __init__(self):
		super(hsr_manipulation_testSM, self).__init__()
		self.name = 'hsr_manipulation_test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:225 y:613, x:1131 y:611
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:14 y:478
		_sm_group_0 = OperatableStateMachine(outcomes=['continue'])

		with _sm_group_0:
			# x:78 y:63
			OperatableStateMachine.add('hsr_JointPoseXtion',
										hsr_JointPose(arm_lift_joint=0.4, arm_flex_joint=-2.169, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=-0.35),
										transitions={'continue': 'hsr_OpenHand'},
										autonomy={'continue': Autonomy.Off})

			# x:617 y:41
			OperatableStateMachine.add('hsr_BoundingBox2D',
										hsr_BoundingBox2D(output_topic="/bounding_box_2d_monitor"),
										transitions={'completed': 'hsr_GraspingPointDetect', 'failed': 'hsr_MoveNeutral'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'detection': 'detection'})

			# x:973 y:46
			OperatableStateMachine.add('hsr_tf2',
										hsr_Tf2(before="head_rgbd_sensor_rgb_frame", after="map"),
										transitions={'continue': 'hsr_PoseDummy', 'failed': 'hsr_MoveNeutral'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'before_pose': 'grasping_point', 'after_pose': 'target_poses'})

			# x:1049 y:220
			OperatableStateMachine.add('hsr_PoseDecision',
										hsr_PoseDecision(),
										transitions={'continue': 'hsr_Move2Center', 'failed': 'hsr_MoveNeutral'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_poses': 'target_poses', 'category': 'dummy_object', 'failed_objects': 'dummy_failed_objects', 'selected_pose_grasp': 'selected_pose_grasp', 'selected_object_name': 'selected_object_name', 'selected_object_status': 'selected_object_status'})

			# x:59 y:411
			OperatableStateMachine.add('hsr_MoveitToPoseGoalAction',
										hsr_MoveitToPoseGoalAction(move_group='whole_body', action_topic='/move_group', tolerance=0.001, orien_tolerance=True),
										transitions={'reached': 'hsr_GrippingObject_close', 'planning_failed': 'hsr_MoveNeutral', 'control_failed': 'hsr_MoveNeutral'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'pose_goal': 'selected_pose_grasp', 'move_status': 'move_status'})

			# x:764 y:42
			OperatableStateMachine.add('hsr_GraspingPointDetect',
										hsr_GraspingPointDetect(output_topic="/bounding_box_2d_monitor", save=True),
										transitions={'completed': 'hsr_tf2', 'failed': 'hsr_MoveNeutral'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'detection': 'detection', 'grasping_point': 'grasping_point'})

			# x:1151 y:42
			OperatableStateMachine.add('hsr_PoseDummy',
										hsr_PoseDummy(),
										transitions={'continue': 'hsr_InitDecision'},
										autonomy={'continue': Autonomy.Off},
										remapping={'dummy_pose': 'dummy_pose', 'dummy_object': 'dummy_object', 'dummy_failed_objects': 'dummy_failed_objects'})

			# x:429 y:40
			OperatableStateMachine.add('hsr_ImageCapture',
										hsr_ImageCapture(rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info"),
										transitions={'completed': 'hsr_BoundingBox2D', 'failed': 'hsr_MoveNeutral'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info'})

			# x:36 y:298
			OperatableStateMachine.add('hsr_ViewMaker_grasp_point',
										hsr_ViewMarker(),
										transitions={'continue': 'hsr_MoveitToPoseGoalAction', 'failed': 'hsr_MoveNeutral'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'view_pose': 'selected_pose_grasp'})

			# x:59 y:489
			OperatableStateMachine.add('hsr_GrippingObject_close',
										hsr_GrippingObject(grasp_force=0.7, mode=True),
										transitions={'continue': 'hsr_JointPose_initial', 'failed': 'hsr_MoveNeutral'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:60 y:566
			OperatableStateMachine.add('hsr_JointPose_initial',
										hsr_JointPose(arm_lift_joint=0.05, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'continue'},
										autonomy={'continue': Autonomy.Off})

			# x:458 y:234
			OperatableStateMachine.add('hsr_CollisionBox',
										hsr_CollisionBox(offset_z=0.1, offset_dist=0.1, width=1.2, mode='C'),
										transitions={'continue': 'hsr_CreateApproach'},
										autonomy={'continue': Autonomy.Off},
										remapping={'box_pose': 'selected_pose_grasp'})

			# x:978 y:533
			OperatableStateMachine.add('hsr_MoveNeutral',
										hsr_MoveNeutral(),
										transitions={'continue': 'continue'},
										autonomy={'continue': Autonomy.Off})

			# x:217 y:72
			OperatableStateMachine.add('hsr_OpenHand',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_ImageCapture', 'failed': 'hsr_MoveNeutral'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:30 y:226
			OperatableStateMachine.add('hsr_MoveitToPoseAppAction',
										hsr_MoveitToPoseGoalAction(move_group='whole_body', action_topic='/move_group', tolerance=0.001, orien_tolerance=True),
										transitions={'reached': 'hsr_ViewMaker_grasp_point', 'planning_failed': 'hsr_MoveNeutral', 'control_failed': 'hsr_MoveNeutral'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'pose_goal': 'selected_pose_approach', 'move_status': 'move_status'})

			# x:604 y:239
			OperatableStateMachine.add('hsr_JointPoseLine',
										hsr_JointPoseLine(line_z=0.0),
										transitions={'continue': 'hsr_CollisionBox'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose_goal': 'selected_pose_grasp'})

			# x:1259 y:247
			OperatableStateMachine.add('hsr_RemoveBox',
										hsr_CollisionBox(offset_z=0.3, offset_dist=0.1, width=0.7, mode='R'),
										transitions={'continue': 'hsr_PoseDecision'},
										autonomy={'continue': Autonomy.Off},
										remapping={'box_pose': 'dummy_pose'})

			# x:211 y:186
			OperatableStateMachine.add('hsr_ViewMarker',
										hsr_ViewMarker(),
										transitions={'continue': 'hsr_MoveitToPoseAppAction', 'failed': 'hsr_MoveNeutral'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'view_pose': 'selected_pose_approach'})

			# x:757 y:237
			OperatableStateMachine.add('hsr_ArmInit',
										hsr_MoveNeutral(),
										transitions={'continue': 'hsr_JointPoseLine'},
										autonomy={'continue': Autonomy.Off})

			# x:889 y:233
			OperatableStateMachine.add('hsr_Move2Center',
										hsr_Move2Center(before='map', after='base_footprint'),
										transitions={'continue': 'hsr_ArmInit', 'failed': 'hsr_MoveNeutral'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose': 'selected_pose_grasp'})

			# x:320 y:214
			OperatableStateMachine.add('hsr_CreateApproach',
										hsr_CreateApproach(offset=0.3),
										transitions={'continue': 'hsr_ViewMarker', 'failed': 'hsr_MoveNeutral'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose': 'selected_pose_grasp', 'selected_pose_approach': 'selected_pose_approach', 'selected_pose_grasp': 'selected_pose_grasp'})

			# x:1243 y:154
			OperatableStateMachine.add('hsr_InitDecision',
										hsr_MoveNeutral(),
										transitions={'continue': 'hsr_RemoveBox'},
										autonomy={'continue': Autonomy.Off})



		with _state_machine:
			# x:444 y:346
			OperatableStateMachine.add('Group',
										_sm_group_0,
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

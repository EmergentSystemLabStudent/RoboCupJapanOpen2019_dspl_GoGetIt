#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_speech_recognition_head_microphone_state import hsr_SpeechRecognitionByHeadMic
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBase
from hsr_flexbe_states.hsr_TidyUpDummy import hsr_TidyUpDummy
from hsr_flexbe_states.hsr_planning_scene_control import hsr_PlanningSceneControl
from hsr_flexbe_states.hsr_xtion_point_state import hsr_XtionPoint
from hsr_flexbe_states.hsr_moveit_to_pose_goal_action_state import hsr_MoveitToPoseGoalAction
from hsr_flexbe_states.hsr_gripping_object_state import hsr_GrippingObject
from hsr_flexbe_states.hsr_reset_octomap import hsr_ResetOctomap
from hsr_flexbe_states.hsr_clear_bbox_octomap import hsr_ClearBBoxOctomap
from hsr_flexbe_states.hsr_joint_pose_state import hsr_JointPose
from hsr_flexbe_states.hsr_omni_base_state import hsr_OmniBase
from hsr_flexbe_states.hsr_image_capture_state import hsr_ImageCapture
from hsr_flexbe_states.hsr_bounding_box_2d_state import hsr_BoundingBox2D
from hsr_flexbe_states.hsr_grasping_point_detect_state import hsr_GraspingPointDetect
from hsr_flexbe_states.hsr_tf2_state import hsr_Tf2
from hsr_flexbe_states.hsr_pose_decision import hsr_PoseDecision
from hsr_flexbe_states.hsr_veiw_marker import hsr_ViewMarker
from hsr_flexbe_states.hsr_tts_input_parameter_state import hsr_TtsInputParameter
from hsr_flexbe_states.hsr_tidy_up_stage2_state import hsr_TidyUpStage2
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Oct 02 2018
@author: tabuchi
'''
class hsr_tidy_up_stage2SM(Behavior):
	'''
	tidy up stage2
	'''


	def __init__(self):
		super(hsr_tidy_up_stage2SM, self).__init__()
		self.name = 'hsr_tidy_up_stage2'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:686 y:318, x:620 y:319
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]

		# x:30 y:514, x:130 y:514, x:230 y:514, x:330 y:514
		_sm_grasp_phase_0 = OperatableStateMachine(outcomes=['succeeded', 'failed', 'planning_failed', 'control_failed'], input_keys=['move_pose', 'xtion_point', 'object', 'failed_objects'], output_keys=['selected_object_name', 'selected_object_status'])

		with _sm_grasp_phase_0:
			# x:32 y:41
			OperatableStateMachine.add('hsr_MoveBase_grasp',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_JointPose_initial', 'failed': 'hsr_MoveBase_grasp'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:1331 y:34
			OperatableStateMachine.add('hsr_XtionPoint_grasp',
										hsr_XtionPoint(ref_frame_id="map"),
										transitions={'finish': 'hsr_ImageCapture'},
										autonomy={'finish': Autonomy.Off},
										remapping={'xtion_point': 'xtion_point'})

			# x:1510 y:35
			OperatableStateMachine.add('hsr_ImageCapture',
										hsr_ImageCapture(rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info"),
										transitions={'completed': 'hsr_BoundingBox2D', 'failed': 'hsr_ImageCapture'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info'})

			# x:1665 y:36
			OperatableStateMachine.add('hsr_BoundingBox2D',
										hsr_BoundingBox2D(output_topic="/bounding_box_2d_monitor"),
										transitions={'completed': 'hsr_GraspingPointDetect', 'failed': 'hsr_BoundingBox2D'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'detection': 'detection', 'grasping_point': 'grasping_point'})

			# x:1832 y:36
			OperatableStateMachine.add('hsr_GraspingPointDetect',
										hsr_GraspingPointDetect(output_topic="/bounding_box_2d_monitor", save=True),
										transitions={'completed': 'hsr_Tf2', 'failed': 'hsr_GraspingPointDetect'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'detection': 'detection', 'grasping_point': 'grasping_point'})

			# x:1861 y:139
			OperatableStateMachine.add('hsr_Tf2',
										hsr_Tf2(before="head_rgbd_sensor_rgb_frame", after="map"),
										transitions={'continue': 'hsr_ClearBBoxOctomap_grasp', 'failed': 'hsr_Tf2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'before_pose': 'grasping_point', 'after_pose': 'after_pose'})

			# x:1261 y:139
			OperatableStateMachine.add('hsr_PoseDecision',
										hsr_PoseDecision(offset=None),
										transitions={'continue': 'hsr_ViewMarker', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_poses': 'after_pose', 'category': 'object', 'failed_objects': 'failed_objects', 'selected_pose_approach': 'selected_pose_approach', 'selected_pose_grasp': 'selected_pose_grasp', 'selected_object_name': 'selected_object_name', 'selected_object_status': 'selected_object_status'})

			# x:931 y:140
			OperatableStateMachine.add('hsr_OpenHand_grasp',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_JointPose_OpenHand', 'failed': 'hsr_OpenHand_grasp'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:514 y:140
			OperatableStateMachine.add('hsr_MoveitToPoseGoalAction_grasp',
										hsr_MoveitToPoseGoalAction(move_group='whole_body', action_topic='/move_group'),
										transitions={'reached': 'hsr_CloseHand', 'planning_failed': 'planning_failed', 'control_failed': 'control_failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'pose_goal': 'selected_pose_grasp', 'move_status': 'selected_object_status'})

			# x:318 y:142
			OperatableStateMachine.add('hsr_CloseHand',
										hsr_GrippingObject(grasp_force=0.7, mode=True),
										transitions={'continue': 'hsr_MoveBase_grasp_back', 'failed': 'hsr_MoveBase_grasp2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'selected_object_status'})

			# x:1167 y:35
			OperatableStateMachine.add('hsr_JointPose_xtion',
										hsr_JointPose(arm_lift_joint=0.4, arm_flex_joint=-2.619, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_XtionPoint_grasp'},
										autonomy={'continue': Autonomy.Off})

			# x:743 y:153
			OperatableStateMachine.add('hsr_JointPose_OpenHand',
										hsr_JointPose(arm_lift_joint=0.4, arm_flex_joint=-3.14/4, arm_roll_joint=0.0, wrist_flex_joint=-3.14/4, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_MoveitToPoseGoalAction_grasp'},
										autonomy={'continue': Autonomy.Off})

			# x:1639 y:138
			OperatableStateMachine.add('hsr_ClearBBoxOctomap_grasp',
										hsr_ClearBBoxOctomap(bbox_param_xy=0.15, bbox_param_z=0.3),
										transitions={'completed': 'hsr_PlanningSceneControl_grasp', 'failed': 'hsr_ClearBBoxOctomap_grasp'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_point': 'after_pose'})

			# x:348 y:38
			OperatableStateMachine.add('hsr_ResetOctomap_grasp',
										hsr_ResetOctomap(play_octomap=True),
										transitions={'completed': 'hsr_JointPose_head_bellow_grasp', 'failed': 'hsr_ResetOctomap_grasp'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1110 y:139
			OperatableStateMachine.add('hsr_ViewMarker',
										hsr_ViewMarker(),
										transitions={'continue': 'hsr_OpenHand_grasp', 'failed': 'hsr_ViewMarker'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'view_pose': 'selected_pose_grasp'})

			# x:1418 y:138
			OperatableStateMachine.add('hsr_PlanningSceneControl_grasp',
										hsr_PlanningSceneControl(update_sw=False),
										transitions={'completed': 'hsr_PoseDecision', 'failed': 'hsr_PlanningSceneControl_grasp'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:30 y:144
			OperatableStateMachine.add('hsr_MoveBase_grasp_back',
										hsr_MoveBase(),
										transitions={'succeeded': 'succeeded', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:190 y:39
			OperatableStateMachine.add('hsr_JointPose_initial',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_ResetOctomap_grasp'},
										autonomy={'continue': Autonomy.Off})

			# x:533 y:39
			OperatableStateMachine.add('hsr_JointPose_head_bellow_grasp',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=-0.7),
										transitions={'continue': 'left_turn_grasp'},
										autonomy={'continue': Autonomy.Off})

			# x:750 y:39
			OperatableStateMachine.add('left_turn_grasp',
										hsr_OmniBase(x=0.0, y=0.0, yaw=3.14/9, time_out=30.0),
										transitions={'finish': 'right_turn_grasp'},
										autonomy={'finish': Autonomy.Off})

			# x:898 y:36
			OperatableStateMachine.add('right_turn_grasp',
										hsr_OmniBase(x=0.0, y=0.0, yaw=-3.14/4.5, time_out=30.0),
										transitions={'finish': 'front_turn_grasp'},
										autonomy={'finish': Autonomy.Off})

			# x:1032 y:39
			OperatableStateMachine.add('front_turn_grasp',
										hsr_OmniBase(x=0.0, y=0.0, yaw=3.14/9, time_out=30.0),
										transitions={'finish': 'hsr_JointPose_xtion'},
										autonomy={'finish': Autonomy.Off})

			# x:191 y:285
			OperatableStateMachine.add('hsr_MoveBase_grasp2',
										hsr_MoveBase(),
										transitions={'succeeded': 'failed', 'failed': 'hsr_MoveBase_grasp2'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})


		# x:543 y:350, x:46 y:336, x:254 y:359
		_sm_put_phase_1 = OperatableStateMachine(outcomes=['control_failed', 'succeeded', 'failed'], input_keys=['move_pose', 'xtion_point', 'arm_pose'], output_keys=['selected_object_name'])

		with _sm_put_phase_1:
			# x:49 y:29
			OperatableStateMachine.add('hsr_MoveBase_put',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_XtionPoint_put', 'failed': 'hsr_MoveBase_put'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:227 y:32
			OperatableStateMachine.add('hsr_XtionPoint_put',
										hsr_XtionPoint(ref_frame_id="map"),
										transitions={'finish': 'hsr_ResetOctomap_put'},
										autonomy={'finish': Autonomy.Off},
										remapping={'xtion_point': 'xtion_point'})

			# x:495 y:139
			OperatableStateMachine.add('hsr_MoveitToPoseGoalAction_put',
										hsr_MoveitToPoseGoalAction(move_group='whole_body', action_topic='/move_group'),
										transitions={'reached': 'hsr_OpenHand_put', 'planning_failed': 'hsr_MoveitToPoseGoalAction_put', 'control_failed': 'control_failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'pose_goal': 'arm_pose', 'move_status': 'selected_object_name'})

			# x:275 y:141
			OperatableStateMachine.add('hsr_OpenHand_put',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_MoveBase_put_back', 'failed': 'hsr_OpenHand_put'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:66 y:142
			OperatableStateMachine.add('hsr_MoveBase_put_back',
										hsr_MoveBase(),
										transitions={'succeeded': 'succeeded', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:436 y:34
			OperatableStateMachine.add('hsr_ResetOctomap_put',
										hsr_ResetOctomap(play_octomap=True),
										transitions={'completed': 'hsr_JointPose_head_bellow_put', 'failed': 'hsr_ResetOctomap_put'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1071 y:135
			OperatableStateMachine.add('hsr_ClearBBoxOctomap_put',
										hsr_ClearBBoxOctomap(bbox_param_xy=0.25, bbox_param_z=0.25),
										transitions={'completed': 'hsr_JointPose_put', 'failed': 'hsr_ClearBBoxOctomap_put'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_point': 'arm_pose'})

			# x:736 y:141
			OperatableStateMachine.add('hsr_PlanningSceneControl_put',
										hsr_PlanningSceneControl(update_sw=False),
										transitions={'completed': 'hsr_MoveitToPoseGoalAction_put', 'failed': 'hsr_PlanningSceneControl_put'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:621 y:30
			OperatableStateMachine.add('hsr_JointPose_head_bellow_put',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=-0.7),
										transitions={'continue': 'left_turn_put'},
										autonomy={'continue': Autonomy.Off})

			# x:842 y:30
			OperatableStateMachine.add('left_turn_put',
										hsr_OmniBase(x=0.0, y=0.0, yaw=3.14/9, time_out=30.0),
										transitions={'finish': 'right_turn_put'},
										autonomy={'finish': Autonomy.Off})

			# x:981 y:30
			OperatableStateMachine.add('right_turn_put',
										hsr_OmniBase(x=0.0, y=0.0, yaw=-3.14/4.5, time_out=30.0),
										transitions={'finish': 'front_turn_put'},
										autonomy={'finish': Autonomy.Off})

			# x:1125 y:26
			OperatableStateMachine.add('front_turn_put',
										hsr_OmniBase(x=0.0, y=0.0, yaw=3.14/9, time_out=30.0),
										transitions={'finish': 'hsr_ClearBBoxOctomap_put'},
										autonomy={'finish': Autonomy.Off})

			# x:934 y:144
			OperatableStateMachine.add('hsr_JointPose_put',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=-3.14/6, arm_roll_joint=0.0, wrist_flex_joint=-3.14/3, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_PlanningSceneControl_put'},
										autonomy={'continue': Autonomy.Off})



		with _state_machine:
			# x:23 y:20
			OperatableStateMachine.add('hsr_SpeechRecognitionByHeadMic',
										hsr_SpeechRecognitionByHeadMic(),
										transitions={'recognition': 'hsr_TidyUpDummy'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:453 y:24
			OperatableStateMachine.add('hsr_MoveBase_kitchen',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_SpeechRecognitionByHeadMic', 'failed': 'hsr_MoveBase_kitchen'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:198 y:99
			OperatableStateMachine.add('hsr_TidyUpDummy',
										hsr_TidyUpDummy(),
										transitions={'continue': 'hsr_TidyUpStage2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'dummy1': 'selected_object_name', 'dummy2': 'selected_object_status'})

			# x:311 y:311
			OperatableStateMachine.add('hsr_PlanningSceneControl_True',
										hsr_PlanningSceneControl(update_sw=True),
										transitions={'completed': 'hsr_TidyUpStage2', 'failed': 'hsr_PlanningSceneControl_True'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:336 y:425
			OperatableStateMachine.add('put_phase',
										_sm_put_phase_1,
										transitions={'control_failed': 'failed', 'succeeded': 'hsr_PlanningSceneControl_True', 'failed': 'hsr_PlanningSceneControl_True'},
										autonomy={'control_failed': Autonomy.Inherit, 'succeeded': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'move_pose': 'move_pose', 'xtion_point': 'xtion_point', 'arm_pose': 'arm_pose', 'selected_object_name': 'selected_object_name'})

			# x:333 y:189
			OperatableStateMachine.add('grasp_phase',
										_sm_grasp_phase_0,
										transitions={'succeeded': 'hsr_PlanningSceneControl_True', 'failed': 'hsr_PlanningSceneControl_True', 'planning_failed': 'hsr_TidyUpStage2', 'control_failed': 'failed'},
										autonomy={'succeeded': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'planning_failed': Autonomy.Inherit, 'control_failed': Autonomy.Inherit},
										remapping={'move_pose': 'move_pose', 'xtion_point': 'xtion_point', 'object': 'object', 'failed_objects': 'failed_objects', 'selected_object_name': 'selected_object_name', 'selected_object_status': 'selected_object_status'})

			# x:9 y:114
			OperatableStateMachine.add('tts_RetrySpeechRecognition',
										hsr_TtsInputParameter(text="Could you repeat one more time?", language="en"),
										transitions={'completed': 'hsr_SpeechRecognitionByHeadMic'},
										autonomy={'completed': Autonomy.Off})

			# x:59 y:278
			OperatableStateMachine.add('hsr_TidyUpStage2',
										hsr_TidyUpStage2(),
										transitions={'retry': 'tts_RetrySpeechRecognition', 'grasp': 'grasp_phase', 'put': 'put_phase', 'kitchen': 'hsr_MoveBase_kitchen'},
										autonomy={'retry': Autonomy.Off, 'grasp': Autonomy.Off, 'put': Autonomy.Off, 'kitchen': Autonomy.Off},
										remapping={'recognition': 'recognition', 'grasp_object': 'selected_object_name', 'grasp_status': 'selected_object_status', 'move_pose': 'move_pose', 'xtion_point': 'xtion_point', 'arm_pose': 'arm_pose', 'object': 'object', 'failed_objects': 'failed_objects'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]

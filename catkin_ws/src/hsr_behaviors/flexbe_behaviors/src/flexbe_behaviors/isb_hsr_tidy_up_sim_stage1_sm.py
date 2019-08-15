#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_speech_recognition_state import hsr_SpeechRecognition
from hsr_flexbe_states.hsr_spacoty_instruction import hsr_SpaCoTyInstruction
from hsr_flexbe_states.hsr_spacoty_get_dataset import hsr_SpaCoTyGetDataset
from hsr_flexbe_states.hsr_spacoty_training_state import hsr_SpaCoTyTraining
from hsr_flexbe_states.hsr_time_sim_start import hsr_TimeSimStart
from hsr_flexbe_states.hsr_tf2_state import hsr_Tf2
from hsr_flexbe_states.hsr_joint_pose_state import hsr_JointPose
from hsr_flexbe_states.hsr_veiw_marker import hsr_ViewMarker
from hsr_flexbe_states.hsr_gripping_object_state import hsr_GrippingObject
from hsr_flexbe_states.hsr_planning_scene_control import hsr_PlanningSceneControl
from hsr_flexbe_states.hsr_image_capture_state import hsr_ImageCapture
from hsr_flexbe_states.hsr_counting import hsr_Counting
from hsr_flexbe_states.hsr_move_base_client import hsr_MoveBaseClient
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBase
from hsr_flexbe_states.hsr_reset_octomap import hsr_ResetOctomap
from hsr_flexbe_states.hsr_grasping_point_detect_state_isb import hsr_GraspingPointDetectIsb
from hsr_flexbe_states.hsr_time_counter_end import hsr_TimeCounterEnd
from hsr_flexbe_states.hsr_spacoty_planning import hsr_SpaCoTyPlanning
from hsr_flexbe_states.hsr_time_sim_end import hsr_TimeSimEnd
from hsr_flexbe_states.hsr_time_counter_start import hsr_TimeCounterStart
from hsr_flexbe_states.hsr_clear_bbox_octomap_one import hsr_ClearBBoxOctomapOne
from hsr_flexbe_states.hsr_omni_base_state import hsr_OmniBase
from hsr_flexbe_states.hsr_clear_bbox_octomap import hsr_ClearBBoxOctomap
from hsr_flexbe_states.hsr_pose_decision_tidy_up import hsr_PoseDecisionTidyUp
from hsr_flexbe_states.hsr_make_file import hsr_MakeFile
from hsr_flexbe_states.hsr_select_tidy_pose_from_train_dataset_of_spacoty import hsr_SelectTidyObjectandPosefromDataset
from hsr_flexbe_states.hsr_select_tidy_pose_randomly_from_train_dataset_of_spacoty import hsr_SelectTidyObjectandPoseRandomly
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Dec 05 2018
@author: Shota Isobe
'''
class isb_hsr_tidy_up_sim_stage1SM(Behavior):
	'''
	[Simulator] This state-machine work for tidying up in the room by my master's work .
	'''


	def __init__(self):
		super(isb_hsr_tidy_up_sim_stage1SM, self).__init__()
		self.name = 'isb_hsr_tidy_up_sim_stage1'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1233 y:679, x:1160 y:680
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]

		# x:20 y:426
		_sm_tidy_baseline_random_0 = OperatableStateMachine(outcomes=['completed'])

		with _sm_tidy_baseline_random_0:
			# x:67 y:25
			OperatableStateMachine.add('hsr_Timer_start',
										hsr_TimeSimStart(sw=True, timer=12),
										transitions={'continue': 'hsr_HandOpen_1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'start_time_sim': 'start_time_sim'})

			# x:1155 y:82
			OperatableStateMachine.add('hsr_Tf2',
										hsr_Tf2(before="head_rgbd_sensor_rgb_frame", after="map"),
										transitions={'continue': 'hsr_SelectTidyObjectandPoseRandomly', 'failed': 'hsr_Tf2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'before_pose': 'grasping_point', 'after_pose': 'after_pose'})

			# x:374 y:229
			OperatableStateMachine.add('hsr_JointPose_Grasp_1',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=-0.7, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=-0.6),
										transitions={'continue': 'hsr_PoseDecisionTidyUp_1'},
										autonomy={'continue': Autonomy.Off})

			# x:398 y:370
			OperatableStateMachine.add('hsr_ViewGraspPoint',
										hsr_ViewMarker(),
										transitions={'continue': 'hsr_HandClose', 'failed': 'hsr_ViewGraspPoint'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'view_pose': 'selected_pose_grasp1'})

			# x:562 y:228
			OperatableStateMachine.add('hsr_HandOpen_2',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_JointPose_Grasp_1', 'failed': 'hsr_HandOpen_2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:697 y:453
			OperatableStateMachine.add('hsr_HandClose',
										hsr_GrippingObject(grasp_force=0.7, mode=True),
										transitions={'continue': 'hsr_JointPose_Go_3', 'failed': 'hsr_JointPose_Go_2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:733 y:227
			OperatableStateMachine.add('hsr_PlanningSceneStop_1',
										hsr_PlanningSceneControl(update_sw=False, wait_time=0.0),
										transitions={'completed': 'hsr_MoveBase_GraspPose', 'failed': 'hsr_PlanningSceneStop_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:794 y:81
			OperatableStateMachine.add('hsr_ImageCapture',
										hsr_ImageCapture(rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info"),
										transitions={'completed': 'hsr_GraspingPointDetectIsb', 'failed': 'hsr_CaptureTime'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info'})

			# x:781 y:149
			OperatableStateMachine.add('hsr_CaptureTime',
										hsr_Counting(max_count=2),
										transitions={'repetition': 'hsr_CaptureTime', 'end': 'hsr_ImageCapture'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'count_num': 'capture_num'})

			# x:746 y:298
			OperatableStateMachine.add('hsr_MoveBase_GraspPose',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=-0.7, pose_orientation_w=0.7),
										transitions={'completed': 'hsr_MoveBaseGP'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'grasp_move_pose'})

			# x:574 y:300
			OperatableStateMachine.add('hsr_MoveBaseGP',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_HandOpen_2', 'failed': 'hsr_MoveBaseGP'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'grasp_move_pose'})

			# x:264 y:12
			OperatableStateMachine.add('hsr_MoveBaseSP_1',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_PlanningSceneStart_1', 'failed': 'hsr_MoveBaseSP_1'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'start_move_pose1'})

			# x:429 y:82
			OperatableStateMachine.add('hsr_JointPose_FindObj',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=-1.3, head_tilt_joint=-0.8),
										transitions={'continue': 'hsr_ResetOctomap_1'},
										autonomy={'continue': Autonomy.Off})

			# x:617 y:80
			OperatableStateMachine.add('hsr_ResetOctomap_1',
										hsr_ResetOctomap(octomap_topic="/octomap_server_dynamic/reset"),
										transitions={'completed': 'hsr_ImageCapture', 'failed': 'hsr_ResetOctomap_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:946 y:299
			OperatableStateMachine.add('hsr_JointPose_Go_1',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=-0.8),
										transitions={'continue': 'hsr_ClearBBoxOctomapOne'},
										autonomy={'continue': Autonomy.Off})

			# x:951 y:83
			OperatableStateMachine.add('hsr_GraspingPointDetectIsb',
										hsr_GraspingPointDetectIsb(output_topic="/bounding_box_2d_monitor", save=True),
										transitions={'completed': 'hsr_Tf2', 'failed': 'hsr_PoseSelect', 'yolo_failed': 'hsr_DetectErrorIgnoreOnce'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off, 'yolo_failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'grasping_point': 'grasping_point'})

			# x:54 y:226
			OperatableStateMachine.add('hsr_MoveBaseStartPose_1',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=-0.15, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseSP_1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'start_move_pose1'})

			# x:70 y:356
			OperatableStateMachine.add('hsr_Timer_end',
										hsr_TimeSimEnd(sw=True, timer=12),
										transitions={'continue': 'hsr_MoveBaseStartPose_1', 'finished': 'completed'},
										autonomy={'continue': Autonomy.Off, 'finished': Autonomy.Off},
										remapping={'start_time_sim': 'start_time_sim'})

			# x:611 y:14
			OperatableStateMachine.add('hsr_MoveBaseSearch_1',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=-0.12, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseSP_1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'start_move_pose1'})

			# x:969 y:15
			OperatableStateMachine.add('hsr_PoseSelect',
										hsr_Counting(max_count=2),
										transitions={'repetition': 'hsr_MoveBaseSearch_1', 'end': 'hsr_MoveBaseSearch_2'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'count_num': 'pose_num'})

			# x:426 y:14
			OperatableStateMachine.add('hsr_MoveBaseSearch_2',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=0.12, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseSP_1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'start_move_pose1'})

			# x:1178 y:452
			OperatableStateMachine.add('hsr_MoveBaseSP_2',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_ResetOctomap_2', 'failed': 'hsr_MoveBaseSP_2'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'start_move_pose2'})

			# x:1003 y:451
			OperatableStateMachine.add('hsr_MoveBaseStartPose_2',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=0.0, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseSP_2'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'start_move_pose2'})

			# x:1174 y:525
			OperatableStateMachine.add('hsr_ResetOctomap_2',
										hsr_ResetOctomap(octomap_topic="/octomap_server_dynamic/reset"),
										transitions={'completed': 'hsr_JointPose_Head_Up_1', 'failed': 'hsr_ResetOctomap_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:997 y:668
			OperatableStateMachine.add('hsr_PlanningSceneStart_2',
										hsr_PlanningSceneControl(update_sw=True, wait_time=0.0),
										transitions={'completed': 'hsr_ClearBBoxOctomap', 'failed': 'hsr_PlanningSceneStart_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:662 y:668
			OperatableStateMachine.add('hsr_PlanningSceneStop_2',
										hsr_PlanningSceneControl(update_sw=False, wait_time=0.3),
										transitions={'completed': 'hsr_PoseDecisionTidyUp_2', 'failed': 'hsr_PlanningSceneStop_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:485 y:669
			OperatableStateMachine.add('hsr_ViewPutPoint',
										hsr_ViewMarker(),
										transitions={'continue': 'hsr_HandOpen_3', 'failed': 'hsr_ViewPutPoint'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'view_pose': 'selected_pose_grasp2'})

			# x:350 y:668
			OperatableStateMachine.add('hsr_HandOpen_3',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_JointPose_Neutral', 'failed': 'hsr_HandOpen_3'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:69 y:463
			OperatableStateMachine.add('hsr_TidyCount',
										hsr_Counting(max_count=15),
										transitions={'repetition': 'hsr_Timer_end', 'end': 'completed'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'count_num': 'tidy_num'})

			# x:250 y:82
			OperatableStateMachine.add('hsr_PlanningSceneStart_1',
										hsr_PlanningSceneControl(update_sw=True, wait_time=0.0),
										transitions={'completed': 'hsr_JointPose_FindObj', 'failed': 'hsr_PlanningSceneStart_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:66 y:110
			OperatableStateMachine.add('hsr_HandOpen_1',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_MoveBaseStartPose_1', 'failed': 'hsr_HandOpen_1'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:834 y:453
			OperatableStateMachine.add('hsr_JointPose_Go_3',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_MoveBaseStartPose_2'},
										autonomy={'continue': Autonomy.Off})

			# x:914 y:228
			OperatableStateMachine.add('hsr_ClearBBoxOctomapOne',
										hsr_ClearBBoxOctomapOne(topic_name="/octomap_server_dynamic/clear_bbx", bbox_param_xy=0.15, bbox_param_z=0.3),
										transitions={'completed': 'hsr_PlanningSceneStop_1', 'failed': 'hsr_ClearBBoxOctomapOne'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_name': 'tidy_name_random', 'object_point': 'after_pose'})

			# x:402 y:449
			OperatableStateMachine.add('hsr_JointPose_Go_2',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_Timer_end'},
										autonomy={'continue': Autonomy.Off})

			# x:187 y:672
			OperatableStateMachine.add('hsr_JointPose_Neutral',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_OmniBase_Back'},
										autonomy={'continue': Autonomy.Off})

			# x:46 y:670
			OperatableStateMachine.add('hsr_OmniBase_Back',
										hsr_OmniBase(x=-0.1, y=0.0, yaw=0.0, time_out=10.0),
										transitions={'finish': 'hsr_TidyCount'},
										autonomy={'finish': Autonomy.Off})

			# x:932 y:150
			OperatableStateMachine.add('hsr_DetectErrorIgnoreOnce',
										hsr_Counting(max_count=2),
										transitions={'repetition': 'hsr_DetectErrorIgnoreOnce', 'end': 'hsr_GraspingPointDetectIsb'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'count_num': 'count_num'})

			# x:837 y:668
			OperatableStateMachine.add('hsr_ClearBBoxOctomap',
										hsr_ClearBBoxOctomap(topic_name="/octomap_server_dynamic/clear_bbx", bbox_param_xy=0.13, bbox_param_z=0.18),
										transitions={'completed': 'hsr_PlanningSceneStop_2', 'failed': 'hsr_ClearBBoxOctomap'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_point': 'tidy_pose_random'})

			# x:1157 y:598
			OperatableStateMachine.add('hsr_JointPose_Head_Up_1',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.2),
										transitions={'continue': 'hsr_JointPose_Head_Up_2'},
										autonomy={'continue': Autonomy.Off})

			# x:1164 y:666
			OperatableStateMachine.add('hsr_JointPose_Head_Up_2',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.43),
										transitions={'continue': 'hsr_PlanningSceneStart_2'},
										autonomy={'continue': Autonomy.Off})

			# x:367 y:301
			OperatableStateMachine.add('hsr_PoseDecisionTidyUp_1',
										hsr_PoseDecisionTidyUp(offset=-0.03, tolerance=0.0001, multi_plan=False, multi_plan_num=3, orien_tolerance=True),
										transitions={'continue': 'hsr_ViewGraspPoint', 'plan_failed': 'hsr_Timer_end', 'detect_failed': 'hsr_Timer_end'},
										autonomy={'continue': Autonomy.Off, 'plan_failed': Autonomy.Off, 'detect_failed': Autonomy.Off},
										remapping={'target_poses': 'after_pose', 'target_object_names': 'tidy_name_random', 'selected_pose_approach': 'selected_pose_approach1', 'selected_pose_grasp': 'selected_pose_grasp1', 'selected_object_name': 'selected_object_name1', 'selected_object_status': 'selected_object_status1'})

			# x:557 y:587
			OperatableStateMachine.add('hsr_PoseDecisionTidyUp_2',
										hsr_PoseDecisionTidyUp(offset=None, tolerance=0.005, multi_plan=True, multi_plan_num=3, orien_tolerance=True),
										transitions={'continue': 'hsr_ViewPutPoint', 'plan_failed': 'hsr_Timer_end', 'detect_failed': 'hsr_Timer_end'},
										autonomy={'continue': Autonomy.Off, 'plan_failed': Autonomy.Off, 'detect_failed': Autonomy.Off},
										remapping={'target_poses': 'tidy_pose_random', 'target_object_names': 'tidy_name_random', 'selected_pose_approach': 'selected_pose_approach2', 'selected_pose_grasp': 'selected_pose_grasp2', 'selected_object_name': 'selected_object_name2', 'selected_object_status': 'selected_object_status2'})

			# x:1091 y:300
			OperatableStateMachine.add('hsr_SelectTidyObjectandPoseRandomly',
										hsr_SelectTidyObjectandPoseRandomly(),
										transitions={'completed': 'hsr_JointPose_Go_1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'detect_objects': 'after_pose', 'tidy_name_random': 'tidy_name_random', 'tidy_pose_random': 'tidy_pose_random'})


		# x:16 y:429
		_sm_tidy_baseline_near_1 = OperatableStateMachine(outcomes=['completed'])

		with _sm_tidy_baseline_near_1:
			# x:67 y:25
			OperatableStateMachine.add('hsr_Timer_start',
										hsr_TimeSimStart(sw=True, timer=12),
										transitions={'continue': 'hsr_HandOpen_1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'start_time_sim': 'start_time_sim'})

			# x:1155 y:82
			OperatableStateMachine.add('hsr_Tf2',
										hsr_Tf2(before="head_rgbd_sensor_rgb_frame", after="map"),
										transitions={'continue': 'hsr_SelectTidyObjectandPosefromDataset', 'failed': 'hsr_Tf2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'before_pose': 'grasping_point', 'after_pose': 'after_pose'})

			# x:374 y:229
			OperatableStateMachine.add('hsr_JointPose_Grasp_1',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=-0.7, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=-0.6),
										transitions={'continue': 'hsr_PoseDecisionTidyUp_1'},
										autonomy={'continue': Autonomy.Off})

			# x:398 y:370
			OperatableStateMachine.add('hsr_ViewGraspPoint',
										hsr_ViewMarker(),
										transitions={'continue': 'hsr_HandClose', 'failed': 'hsr_ViewGraspPoint'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'view_pose': 'selected_pose_grasp1'})

			# x:562 y:228
			OperatableStateMachine.add('hsr_HandOpen_2',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_JointPose_Grasp_1', 'failed': 'hsr_HandOpen_2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:697 y:453
			OperatableStateMachine.add('hsr_HandClose',
										hsr_GrippingObject(grasp_force=0.7, mode=True),
										transitions={'continue': 'hsr_JointPose_Go_3', 'failed': 'hsr_JointPose_Go_2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:733 y:227
			OperatableStateMachine.add('hsr_PlanningSceneStop_1',
										hsr_PlanningSceneControl(update_sw=False, wait_time=0.0),
										transitions={'completed': 'hsr_MoveBase_GraspPose', 'failed': 'hsr_PlanningSceneStop_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:794 y:81
			OperatableStateMachine.add('hsr_ImageCapture',
										hsr_ImageCapture(rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info"),
										transitions={'completed': 'hsr_GraspingPointDetectIsb', 'failed': 'hsr_CaptureTime'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info'})

			# x:781 y:149
			OperatableStateMachine.add('hsr_CaptureTime',
										hsr_Counting(max_count=2),
										transitions={'repetition': 'hsr_CaptureTime', 'end': 'hsr_ImageCapture'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'count_num': 'capture_num'})

			# x:746 y:298
			OperatableStateMachine.add('hsr_MoveBase_GraspPose',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=-0.7, pose_orientation_w=0.7),
										transitions={'completed': 'hsr_MoveBaseGP'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'grasp_move_pose'})

			# x:574 y:300
			OperatableStateMachine.add('hsr_MoveBaseGP',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_HandOpen_2', 'failed': 'hsr_MoveBaseGP'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'grasp_move_pose'})

			# x:264 y:12
			OperatableStateMachine.add('hsr_MoveBaseSP_1',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_PlanningSceneStart_1', 'failed': 'hsr_MoveBaseSP_1'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'start_move_pose1'})

			# x:429 y:82
			OperatableStateMachine.add('hsr_JointPose_FindObj',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=-1.3, head_tilt_joint=-0.8),
										transitions={'continue': 'hsr_ResetOctomap_1'},
										autonomy={'continue': Autonomy.Off})

			# x:617 y:80
			OperatableStateMachine.add('hsr_ResetOctomap_1',
										hsr_ResetOctomap(octomap_topic="/octomap_server_dynamic/reset"),
										transitions={'completed': 'hsr_ImageCapture', 'failed': 'hsr_ResetOctomap_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:946 y:299
			OperatableStateMachine.add('hsr_JointPose_Go_1',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=-0.8),
										transitions={'continue': 'hsr_ClearBBoxOctomapOne'},
										autonomy={'continue': Autonomy.Off})

			# x:951 y:83
			OperatableStateMachine.add('hsr_GraspingPointDetectIsb',
										hsr_GraspingPointDetectIsb(output_topic="/bounding_box_2d_monitor", save=True),
										transitions={'completed': 'hsr_Tf2', 'failed': 'hsr_PoseSelect', 'yolo_failed': 'hsr_DetectErrorIgnoreOnce'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off, 'yolo_failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'grasping_point': 'grasping_point'})

			# x:54 y:226
			OperatableStateMachine.add('hsr_MoveBaseStartPose_1',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=-0.15, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseSP_1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'start_move_pose1'})

			# x:70 y:356
			OperatableStateMachine.add('hsr_Timer_end',
										hsr_TimeSimEnd(sw=True, timer=12),
										transitions={'continue': 'hsr_MoveBaseStartPose_1', 'finished': 'completed'},
										autonomy={'continue': Autonomy.Off, 'finished': Autonomy.Off},
										remapping={'start_time_sim': 'start_time_sim'})

			# x:611 y:14
			OperatableStateMachine.add('hsr_MoveBaseSearch_1',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=-0.12, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseSP_1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'start_move_pose1'})

			# x:969 y:15
			OperatableStateMachine.add('hsr_PoseSelect',
										hsr_Counting(max_count=2),
										transitions={'repetition': 'hsr_MoveBaseSearch_1', 'end': 'hsr_MoveBaseSearch_2'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'count_num': 'pose_num'})

			# x:426 y:14
			OperatableStateMachine.add('hsr_MoveBaseSearch_2',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=0.12, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseSP_1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'start_move_pose1'})

			# x:1178 y:452
			OperatableStateMachine.add('hsr_MoveBaseSP_2',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_ResetOctomap_2', 'failed': 'hsr_MoveBaseSP_2'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'start_move_pose2'})

			# x:1003 y:451
			OperatableStateMachine.add('hsr_MoveBaseStartPose_2',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=0.0, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseSP_2'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'start_move_pose2'})

			# x:1174 y:525
			OperatableStateMachine.add('hsr_ResetOctomap_2',
										hsr_ResetOctomap(octomap_topic="/octomap_server_dynamic/reset"),
										transitions={'completed': 'hsr_JointPose_Head_Up_1', 'failed': 'hsr_ResetOctomap_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:997 y:668
			OperatableStateMachine.add('hsr_PlanningSceneStart_2',
										hsr_PlanningSceneControl(update_sw=True, wait_time=0.0),
										transitions={'completed': 'hsr_ClearBBoxOctomap', 'failed': 'hsr_PlanningSceneStart_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:662 y:668
			OperatableStateMachine.add('hsr_PlanningSceneStop_2',
										hsr_PlanningSceneControl(update_sw=False, wait_time=0.3),
										transitions={'completed': 'hsr_PoseDecisionTidyUp_2', 'failed': 'hsr_PlanningSceneStop_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:485 y:669
			OperatableStateMachine.add('hsr_ViewPutPoint',
										hsr_ViewMarker(),
										transitions={'continue': 'hsr_HandOpen_3', 'failed': 'hsr_ViewPutPoint'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'view_pose': 'selected_pose_grasp2'})

			# x:350 y:668
			OperatableStateMachine.add('hsr_HandOpen_3',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_JointPose_Neutral', 'failed': 'hsr_HandOpen_3'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:69 y:463
			OperatableStateMachine.add('hsr_TidyCount',
										hsr_Counting(max_count=15),
										transitions={'repetition': 'hsr_Timer_end', 'end': 'completed'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'count_num': 'tidy_num'})

			# x:250 y:82
			OperatableStateMachine.add('hsr_PlanningSceneStart_1',
										hsr_PlanningSceneControl(update_sw=True, wait_time=0.0),
										transitions={'completed': 'hsr_JointPose_FindObj', 'failed': 'hsr_PlanningSceneStart_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:66 y:110
			OperatableStateMachine.add('hsr_HandOpen_1',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_MoveBaseStartPose_1', 'failed': 'hsr_HandOpen_1'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:834 y:453
			OperatableStateMachine.add('hsr_JointPose_Go_3',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_MoveBaseStartPose_2'},
										autonomy={'continue': Autonomy.Off})

			# x:914 y:228
			OperatableStateMachine.add('hsr_ClearBBoxOctomapOne',
										hsr_ClearBBoxOctomapOne(topic_name="/octomap_server_dynamic/clear_bbx", bbox_param_xy=0.15, bbox_param_z=0.3),
										transitions={'completed': 'hsr_PlanningSceneStop_1', 'failed': 'hsr_ClearBBoxOctomapOne'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_name': 'tidy_name_db', 'object_point': 'after_pose'})

			# x:402 y:449
			OperatableStateMachine.add('hsr_JointPose_Go_2',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_Timer_end'},
										autonomy={'continue': Autonomy.Off})

			# x:187 y:672
			OperatableStateMachine.add('hsr_JointPose_Neutral',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_OmniBase_Back'},
										autonomy={'continue': Autonomy.Off})

			# x:46 y:670
			OperatableStateMachine.add('hsr_OmniBase_Back',
										hsr_OmniBase(x=-0.1, y=0.0, yaw=0.0, time_out=10.0),
										transitions={'finish': 'hsr_TidyCount'},
										autonomy={'finish': Autonomy.Off})

			# x:932 y:150
			OperatableStateMachine.add('hsr_DetectErrorIgnoreOnce',
										hsr_Counting(max_count=2),
										transitions={'repetition': 'hsr_DetectErrorIgnoreOnce', 'end': 'hsr_GraspingPointDetectIsb'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'count_num': 'count_num'})

			# x:837 y:668
			OperatableStateMachine.add('hsr_ClearBBoxOctomap',
										hsr_ClearBBoxOctomap(topic_name="/octomap_server_dynamic/clear_bbx", bbox_param_xy=0.13, bbox_param_z=0.18),
										transitions={'completed': 'hsr_PlanningSceneStop_2', 'failed': 'hsr_ClearBBoxOctomap'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_point': 'tidy_pose_db'})

			# x:1157 y:598
			OperatableStateMachine.add('hsr_JointPose_Head_Up_1',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.2),
										transitions={'continue': 'hsr_JointPose_Head_Up_2'},
										autonomy={'continue': Autonomy.Off})

			# x:1164 y:666
			OperatableStateMachine.add('hsr_JointPose_Head_Up_2',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.43),
										transitions={'continue': 'hsr_PlanningSceneStart_2'},
										autonomy={'continue': Autonomy.Off})

			# x:367 y:301
			OperatableStateMachine.add('hsr_PoseDecisionTidyUp_1',
										hsr_PoseDecisionTidyUp(offset=-0.03, tolerance=0.0001, multi_plan=False, multi_plan_num=3, orien_tolerance=True),
										transitions={'continue': 'hsr_ViewGraspPoint', 'plan_failed': 'hsr_Timer_end', 'detect_failed': 'hsr_Timer_end'},
										autonomy={'continue': Autonomy.Off, 'plan_failed': Autonomy.Off, 'detect_failed': Autonomy.Off},
										remapping={'target_poses': 'after_pose', 'target_object_names': 'tidy_name_db', 'selected_pose_approach': 'selected_pose_approach1', 'selected_pose_grasp': 'selected_pose_grasp1', 'selected_object_name': 'selected_object_name1', 'selected_object_status': 'selected_object_status1'})

			# x:557 y:587
			OperatableStateMachine.add('hsr_PoseDecisionTidyUp_2',
										hsr_PoseDecisionTidyUp(offset=None, tolerance=0.005, multi_plan=True, multi_plan_num=3, orien_tolerance=True),
										transitions={'continue': 'hsr_ViewPutPoint', 'plan_failed': 'hsr_Timer_end', 'detect_failed': 'hsr_Timer_end'},
										autonomy={'continue': Autonomy.Off, 'plan_failed': Autonomy.Off, 'detect_failed': Autonomy.Off},
										remapping={'target_poses': 'tidy_pose_db', 'target_object_names': 'tidy_name_db', 'selected_pose_approach': 'selected_pose_approach2', 'selected_pose_grasp': 'selected_pose_grasp2', 'selected_object_name': 'selected_object_name2', 'selected_object_status': 'selected_object_status2'})

			# x:1089 y:300
			OperatableStateMachine.add('hsr_SelectTidyObjectandPosefromDataset',
										hsr_SelectTidyObjectandPosefromDataset(),
										transitions={'completed': 'hsr_JointPose_Go_1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'detect_objects': 'after_pose', 'tidy_name_db': 'tidy_name_db', 'tidy_pose_db': 'tidy_pose_db'})


		# x:15 y:424
		_sm_tidy_greedy_2 = OperatableStateMachine(outcomes=['completed'])

		with _sm_tidy_greedy_2:
			# x:67 y:25
			OperatableStateMachine.add('hsr_Timer_start',
										hsr_TimeSimStart(sw=True, timer=12),
										transitions={'continue': 'hsr_HandOpen_1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'start_time_sim': 'start_time_sim'})

			# x:1155 y:82
			OperatableStateMachine.add('hsr_Tf2',
										hsr_Tf2(before="head_rgbd_sensor_rgb_frame", after="map"),
										transitions={'continue': 'hsr_MakeFile', 'failed': 'hsr_Tf2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'before_pose': 'grasping_point', 'after_pose': 'after_pose'})

			# x:374 y:229
			OperatableStateMachine.add('hsr_JointPose_Grasp_1',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=-0.7, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=-0.6),
										transitions={'continue': 'hsr_PoseDecisionTidyUp_1'},
										autonomy={'continue': Autonomy.Off})

			# x:398 y:370
			OperatableStateMachine.add('hsr_ViewGraspPoint',
										hsr_ViewMarker(),
										transitions={'continue': 'hsr_HandClose', 'failed': 'hsr_ViewGraspPoint'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'view_pose': 'selected_pose_grasp1'})

			# x:562 y:228
			OperatableStateMachine.add('hsr_HandOpen_2',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_JointPose_Grasp_1', 'failed': 'hsr_HandOpen_2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:697 y:453
			OperatableStateMachine.add('hsr_HandClose',
										hsr_GrippingObject(grasp_force=0.7, mode=True),
										transitions={'continue': 'hsr_JointPose_Go_3', 'failed': 'hsr_JointPose_Go_2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:733 y:227
			OperatableStateMachine.add('hsr_PlanningSceneStop_1',
										hsr_PlanningSceneControl(update_sw=False, wait_time=0.0),
										transitions={'completed': 'hsr_MoveBase_GraspPose', 'failed': 'hsr_PlanningSceneStop_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:794 y:81
			OperatableStateMachine.add('hsr_ImageCapture',
										hsr_ImageCapture(rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info"),
										transitions={'completed': 'hsr_GraspingPointDetectIsb', 'failed': 'hsr_CaptureTime'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info'})

			# x:781 y:149
			OperatableStateMachine.add('hsr_CaptureTime',
										hsr_Counting(max_count=2),
										transitions={'repetition': 'hsr_CaptureTime', 'end': 'hsr_ImageCapture'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'count_num': 'capture_num'})

			# x:746 y:298
			OperatableStateMachine.add('hsr_MoveBase_GraspPose',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=-0.7, pose_orientation_w=0.7),
										transitions={'completed': 'hsr_MoveBaseGP'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'grasp_move_pose'})

			# x:574 y:300
			OperatableStateMachine.add('hsr_MoveBaseGP',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_HandOpen_2', 'failed': 'hsr_MoveBaseGP'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'grasp_move_pose'})

			# x:264 y:12
			OperatableStateMachine.add('hsr_MoveBaseSP_1',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_PlanningSceneStart_1', 'failed': 'hsr_MoveBaseSP_1'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'start_move_pose1'})

			# x:429 y:82
			OperatableStateMachine.add('hsr_JointPose_FindObj',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=-1.3, head_tilt_joint=-0.8),
										transitions={'continue': 'hsr_ResetOctomap_1'},
										autonomy={'continue': Autonomy.Off})

			# x:617 y:80
			OperatableStateMachine.add('hsr_ResetOctomap_1',
										hsr_ResetOctomap(octomap_topic="/octomap_server_dynamic/reset"),
										transitions={'completed': 'hsr_ImageCapture', 'failed': 'hsr_ResetOctomap_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:946 y:299
			OperatableStateMachine.add('hsr_JointPose_Go_1',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=-0.8),
										transitions={'continue': 'hsr_ClearBBoxOctomapOne'},
										autonomy={'continue': Autonomy.Off})

			# x:951 y:83
			OperatableStateMachine.add('hsr_GraspingPointDetectIsb',
										hsr_GraspingPointDetectIsb(output_topic="/bounding_box_2d_monitor", save=True),
										transitions={'completed': 'hsr_Tf2', 'failed': 'hsr_PoseSelect', 'yolo_failed': 'hsr_DetectErrorIgnoreOnce'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off, 'yolo_failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'grasping_point': 'grasping_point'})

			# x:1140 y:350
			OperatableStateMachine.add('hsr_TimeCounter_End',
										hsr_TimeCounterEnd(switch="e"),
										transitions={'completed': 'hsr_JointPose_Go_1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'start_time': 'start_time'})

			# x:1141 y:275
			OperatableStateMachine.add('hsr_SpaCoTyPlanning',
										hsr_SpaCoTyPlanning(),
										transitions={'completed': 'hsr_TimeCounter_End'},
										autonomy={'completed': Autonomy.Off},
										remapping={'load_file': 'load_file', 'detect_objects': 'after_pose', 'tidy_pose': 'tidy_pose', 'tidy_order': 'tidy_order', 'name2place': 'name2place'})

			# x:54 y:226
			OperatableStateMachine.add('hsr_MoveBaseStartPose_1',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=-0.15, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseSP_1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'start_move_pose1'})

			# x:70 y:356
			OperatableStateMachine.add('hsr_Timer_end',
										hsr_TimeSimEnd(sw=True, timer=12),
										transitions={'continue': 'hsr_MoveBaseStartPose_1', 'finished': 'completed'},
										autonomy={'continue': Autonomy.Off, 'finished': Autonomy.Off},
										remapping={'start_time_sim': 'start_time_sim'})

			# x:1136 y:205
			OperatableStateMachine.add('hsr_TimeCounter_start',
										hsr_TimeCounterStart(switch="s"),
										transitions={'completed': 'hsr_SpaCoTyPlanning'},
										autonomy={'completed': Autonomy.Off},
										remapping={'start_time': 'start_time'})

			# x:611 y:14
			OperatableStateMachine.add('hsr_MoveBaseSearch_1',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=-0.12, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseSP_1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'start_move_pose1'})

			# x:969 y:15
			OperatableStateMachine.add('hsr_PoseSelect',
										hsr_Counting(max_count=2),
										transitions={'repetition': 'hsr_MoveBaseSearch_1', 'end': 'hsr_MoveBaseSearch_2'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'count_num': 'pose_num'})

			# x:426 y:14
			OperatableStateMachine.add('hsr_MoveBaseSearch_2',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=0.12, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseSP_1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'start_move_pose1'})

			# x:1178 y:452
			OperatableStateMachine.add('hsr_MoveBaseSP_2',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_ResetOctomap_2', 'failed': 'hsr_MoveBaseSP_2'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'start_move_pose2'})

			# x:1003 y:451
			OperatableStateMachine.add('hsr_MoveBaseStartPose_2',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=0.0, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBaseSP_2'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'start_move_pose2'})

			# x:1174 y:525
			OperatableStateMachine.add('hsr_ResetOctomap_2',
										hsr_ResetOctomap(octomap_topic="/octomap_server_dynamic/reset"),
										transitions={'completed': 'hsr_JointPose_Head_Up_1', 'failed': 'hsr_ResetOctomap_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:997 y:668
			OperatableStateMachine.add('hsr_PlanningSceneStart_2',
										hsr_PlanningSceneControl(update_sw=True, wait_time=0.0),
										transitions={'completed': 'hsr_ClearBBoxOctomap', 'failed': 'hsr_PlanningSceneStart_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:662 y:668
			OperatableStateMachine.add('hsr_PlanningSceneStop_2',
										hsr_PlanningSceneControl(update_sw=False, wait_time=0.3),
										transitions={'completed': 'hsr_PoseDecisionTidyUp_2', 'failed': 'hsr_PlanningSceneStop_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:485 y:669
			OperatableStateMachine.add('hsr_ViewPutPoint',
										hsr_ViewMarker(),
										transitions={'continue': 'hsr_HandOpen_3', 'failed': 'hsr_ViewPutPoint'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'view_pose': 'selected_pose_grasp2'})

			# x:350 y:668
			OperatableStateMachine.add('hsr_HandOpen_3',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_JointPose_Neutral', 'failed': 'hsr_HandOpen_3'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:69 y:463
			OperatableStateMachine.add('hsr_TidyCount',
										hsr_Counting(max_count=15),
										transitions={'repetition': 'hsr_Timer_end', 'end': 'completed'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'count_num': 'tidy_num'})

			# x:250 y:82
			OperatableStateMachine.add('hsr_PlanningSceneStart_1',
										hsr_PlanningSceneControl(update_sw=True, wait_time=0.0),
										transitions={'completed': 'hsr_JointPose_FindObj', 'failed': 'hsr_PlanningSceneStart_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:66 y:110
			OperatableStateMachine.add('hsr_HandOpen_1',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_MoveBaseStartPose_1', 'failed': 'hsr_HandOpen_1'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:834 y:453
			OperatableStateMachine.add('hsr_JointPose_Go_3',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_MoveBaseStartPose_2'},
										autonomy={'continue': Autonomy.Off})

			# x:914 y:228
			OperatableStateMachine.add('hsr_ClearBBoxOctomapOne',
										hsr_ClearBBoxOctomapOne(topic_name="/octomap_server_dynamic/clear_bbx", bbox_param_xy=0.15, bbox_param_z=0.3),
										transitions={'completed': 'hsr_PlanningSceneStop_1', 'failed': 'hsr_ClearBBoxOctomapOne'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_name': 'tidy_order', 'object_point': 'after_pose'})

			# x:402 y:449
			OperatableStateMachine.add('hsr_JointPose_Go_2',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_Timer_end'},
										autonomy={'continue': Autonomy.Off})

			# x:187 y:672
			OperatableStateMachine.add('hsr_JointPose_Neutral',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_OmniBase_Back'},
										autonomy={'continue': Autonomy.Off})

			# x:46 y:670
			OperatableStateMachine.add('hsr_OmniBase_Back',
										hsr_OmniBase(x=-0.1, y=0.0, yaw=0.0, time_out=10.0),
										transitions={'finish': 'hsr_TidyCount'},
										autonomy={'finish': Autonomy.Off})

			# x:932 y:150
			OperatableStateMachine.add('hsr_DetectErrorIgnoreOnce',
										hsr_Counting(max_count=2),
										transitions={'repetition': 'hsr_DetectErrorIgnoreOnce', 'end': 'hsr_GraspingPointDetectIsb'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'count_num': 'count_num'})

			# x:837 y:668
			OperatableStateMachine.add('hsr_ClearBBoxOctomap',
										hsr_ClearBBoxOctomap(topic_name="/octomap_server_dynamic/clear_bbx", bbox_param_xy=0.13, bbox_param_z=0.18),
										transitions={'completed': 'hsr_PlanningSceneStop_2', 'failed': 'hsr_ClearBBoxOctomap'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_point': 'tidy_pose'})

			# x:1157 y:598
			OperatableStateMachine.add('hsr_JointPose_Head_Up_1',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.2),
										transitions={'continue': 'hsr_JointPose_Head_Up_2'},
										autonomy={'continue': Autonomy.Off})

			# x:1164 y:666
			OperatableStateMachine.add('hsr_JointPose_Head_Up_2',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.43),
										transitions={'continue': 'hsr_PlanningSceneStart_2'},
										autonomy={'continue': Autonomy.Off})

			# x:367 y:301
			OperatableStateMachine.add('hsr_PoseDecisionTidyUp_1',
										hsr_PoseDecisionTidyUp(offset=-0.03, tolerance=0.0001, multi_plan=False, multi_plan_num=3, orien_tolerance=True),
										transitions={'continue': 'hsr_ViewGraspPoint', 'plan_failed': 'hsr_Timer_end', 'detect_failed': 'hsr_Timer_end'},
										autonomy={'continue': Autonomy.Off, 'plan_failed': Autonomy.Off, 'detect_failed': Autonomy.Off},
										remapping={'target_poses': 'after_pose', 'target_object_names': 'tidy_order', 'selected_pose_approach': 'selected_pose_approach1', 'selected_pose_grasp': 'selected_pose_grasp1', 'selected_object_name': 'selected_object_name1', 'selected_object_status': 'selected_object_status1'})

			# x:557 y:587
			OperatableStateMachine.add('hsr_PoseDecisionTidyUp_2',
										hsr_PoseDecisionTidyUp(offset=None, tolerance=0.005, multi_plan=True, multi_plan_num=3, orien_tolerance=True),
										transitions={'continue': 'hsr_ViewPutPoint', 'plan_failed': 'hsr_Timer_end', 'detect_failed': 'hsr_Timer_end'},
										autonomy={'continue': Autonomy.Off, 'plan_failed': Autonomy.Off, 'detect_failed': Autonomy.Off},
										remapping={'target_poses': 'tidy_pose', 'target_object_names': 'tidy_order', 'selected_pose_approach': 'selected_pose_approach2', 'selected_pose_grasp': 'selected_pose_grasp2', 'selected_object_name': 'selected_object_name2', 'selected_object_status': 'selected_object_status2'})

			# x:1159 y:145
			OperatableStateMachine.add('hsr_MakeFile',
										hsr_MakeFile(save_file="sim_st1_greedy"),
										transitions={'completed': 'hsr_TimeCounter_start'},
										autonomy={'completed': Autonomy.Off},
										remapping={'load_file': 'load_file'})



		with _state_machine:
			# x:125 y:230
			OperatableStateMachine.add('hsr_SpeechRecognition',
										hsr_SpeechRecognition(topic="/julius/speech2text/en"),
										transitions={'recognition': 'hsr_SpaCoTyInstruction'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'sentence'})

			# x:999 y:231
			OperatableStateMachine.add('hsr_SpaCoTyInstruction',
										hsr_SpaCoTyInstruction(save_folder="sim_st1"),
										transitions={'datadata': 'hsr_SpaCoTyGetDataset', 'training': 'hsr_SpaCoTyTraining', 'tidy_baseline_nearly': 'tidy_baseline_near', 'tidy_baseline_randomly': 'tidy_baseline_random', 'tidy_greedy': 'tidy_greedy', 'failed': 'hsr_SpeechRecognition', 'end': 'finished'},
										autonomy={'datadata': Autonomy.Off, 'training': Autonomy.Off, 'tidy_baseline_nearly': Autonomy.Off, 'tidy_baseline_randomly': Autonomy.Off, 'tidy_greedy': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'sentence': 'sentence', 'move_name': 'move_name', 'save_folder': 'save_folder'})

			# x:577 y:141
			OperatableStateMachine.add('hsr_SpaCoTyGetDataset',
										hsr_SpaCoTyGetDataset(output_topic="/bounding_box_2d_monitor", rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info", yolo_yaml="yolov3-isb-tidy.json", save_image=True, z_offset=0.4, MEMO="/spacoty/get_dataset"),
										transitions={'completed': 'hsr_SpeechRecognition', 'failed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'save_folder': 'save_folder'})

			# x:584 y:55
			OperatableStateMachine.add('hsr_SpaCoTyTraining',
										hsr_SpaCoTyTraining(),
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Off},
										remapping={'save_folder': 'save_folder', 'spacoty_text': 'spacoty_text'})

			# x:582 y:388
			OperatableStateMachine.add('tidy_greedy',
										_sm_tidy_greedy_2,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit})

			# x:578 y:483
			OperatableStateMachine.add('tidy_baseline_near',
										_sm_tidy_baseline_near_1,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit})

			# x:573 y:588
			OperatableStateMachine.add('tidy_baseline_random',
										_sm_tidy_baseline_random_0,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]

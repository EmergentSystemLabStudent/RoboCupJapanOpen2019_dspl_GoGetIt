#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_door_detect_state import hsr_DoorDetect
from hsr_flexbe_states.hsr_tts_input_parameter_state import hsr_TtsInputParameter
from hsr_flexbe_states.hsr_speech_recognition_state import hsr_SpeechRecognition
from hsr_flexbe_states.hsr_joint_pose_state import hsr_JointPose
from hsr_flexbe_states.hsr_label_arrangement_state import hsr_LabelArrangement
from hsr_flexbe_states.hsr_bounding_box_2d_state import hsr_BoundingBox2D
from hsr_flexbe_states.hsr_image_capture_state import hsr_ImageCapture
from hsr_flexbe_states.hsr_move_to_neutral import hsr_MoveNeutral
from hsr_flexbe_states.hsr_spcof_add_data_state import hsr_SpcofAddData
from hsr_flexbe_states.hsr_tts_input_key_state import hsr_TtsInputKey
from hsr_flexbe_states.hsr_follow_me_state import hsr_FollowMe
from hsr_flexbe_states.hsr_ask_place_state import hsr_AskPlace
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBase
from hsr_flexbe_states.hsr_try_limit_state import hsr_try_limit
from hsr_flexbe_states.hsr_yes_no_ask_state import hsr_YesNoAsk
from hsr_flexbe_states.hsr_spcof_move_state import hsr_SpcofMove
from hsr_flexbe_states.hsr_ask_object_state import hsr_AskObject
from hsr_flexbe_states.hsr_read_task_state import hsr_ReadTask
from hsr_flexbe_states.hsr_tf2_state import hsr_Tf2
from hsr_flexbe_states.hsr_pose_decision import hsr_PoseDecision
from hsr_flexbe_states.hsr_moveit_to_pose_goal_action_state import hsr_MoveitToPoseGoalAction
from hsr_flexbe_states.hsr_grasping_point_detect_state import hsr_GraspingPointDetect
from hsr_flexbe_states.hsr_pose_dummy import hsr_PoseDummy
from hsr_flexbe_states.hsr_veiw_marker import hsr_ViewMarker
from hsr_flexbe_states.hsr_gripping_object_state import hsr_GrippingObject
from hsr_flexbe_states.hsr_collision_box_state import hsr_CollisionBox
from hsr_flexbe_states.hsr_joint_pose_line_state import hsr_JointPoseLine
from hsr_flexbe_states.hsr_create_approach import hsr_CreateApproach
from hsr_flexbe_states.hsr_move2center import hsr_Move2Center
from hsr_flexbe_states.hsr_go_get_it_learn_state import hsr_Go_Get_It_Learn
from hsr_flexbe_states.hsr_memorize_start_pos_state import hsr_MemorizeStartPos
from hsr_flexbe_states.hsr_dummy_initial_pos_state import hsr_DummyInitialPos
from hsr_flexbe_states.hsr_spcof_learn_state import hsr_SpcofLearn
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 18 2019
@author: Hitoshi Nakamura
'''
class Go_get_it_unknown_environment2019SM(Behavior):
	'''
	Go_get_it_unknown_environment2019
	'''


	def __init__(self):
		super(Go_get_it_unknown_environment2019SM, self).__init__()
		self.name = 'Go_get_it_unknown_environment2019'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:192 y:383, x:188 y:466
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:309 y:787
		_sm_manipulation_0 = OperatableStateMachine(outcomes=['continue'], input_keys=['object_name', 'move_pose'])

		with _sm_manipulation_0:
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

			# x:949 y:59
			OperatableStateMachine.add('hsr_tf2',
										hsr_Tf2(before="head_rgbd_sensor_rgb_frame", after="map"),
										transitions={'continue': 'hsr_PoseDummy', 'failed': 'hsr_MoveNeutral'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'before_pose': 'grasping_point', 'after_pose': 'target_poses'})

			# x:1094 y:250
			OperatableStateMachine.add('hsr_PoseDecision',
										hsr_PoseDecision(),
										transitions={'continue': 'hsr_ArmInit', 'failed': 'hsr_MoveNeutral'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_poses': 'target_poses', 'category': 'object_name', 'failed_objects': 'dummy_failed_objects', 'selected_pose_grasp': 'selected_pose_grasp', 'selected_object_name': 'selected_object_name', 'selected_object_status': 'selected_object_status'})

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

			# x:1270 y:64
			OperatableStateMachine.add('hsr_PoseDummy',
										hsr_PoseDummy(),
										transitions={'continue': 'hsr_RemoveBox'},
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
										transitions={'continue': 'MoveInitPosition', 'failed': 'hsr_MoveNeutral'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:94 y:720
			OperatableStateMachine.add('hsr_JointPose_initial',
										hsr_JointPose(arm_lift_joint=0.05, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'continue'},
										autonomy={'continue': Autonomy.Off})

			# x:503 y:237
			OperatableStateMachine.add('hsr_CollisionBox',
										hsr_CollisionBox(offset_z=0.1, offset_dist=0.1, width=1.2, mode='C'),
										transitions={'continue': 'hsr_CreateApproach'},
										autonomy={'continue': Autonomy.Off},
										remapping={'box_pose': 'selected_pose_grasp'})

			# x:758 y:653
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

			# x:804 y:239
			OperatableStateMachine.add('hsr_JointPoseLine',
										hsr_JointPoseLine(line_z=0.0),
										transitions={'continue': 'hsr_Move2Center'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose_goal': 'selected_pose_grasp'})

			# x:1279 y:245
			OperatableStateMachine.add('hsr_RemoveBox',
										hsr_CollisionBox(offset_z=0.3, offset_dist=0.1, width=0.7, mode='R'),
										transitions={'continue': 'hsr_InitDecision'},
										autonomy={'continue': Autonomy.Off},
										remapping={'box_pose': 'dummy_pose'})

			# x:271 y:224
			OperatableStateMachine.add('hsr_ViewMarker',
										hsr_ViewMarker(),
										transitions={'continue': 'hsr_MoveitToPoseAppAction', 'failed': 'hsr_MoveNeutral'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'view_pose': 'selected_pose_approach'})

			# x:939 y:230
			OperatableStateMachine.add('hsr_ArmInit',
										hsr_MoveNeutral(),
										transitions={'continue': 'hsr_JointPoseLine'},
										autonomy={'continue': Autonomy.Off})

			# x:112 y:644
			OperatableStateMachine.add('MoveInitPosition',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_JointPose_initial', 'failed': 'hsr_MoveNeutral'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:1232 y:339
			OperatableStateMachine.add('hsr_InitDecision',
										hsr_MoveNeutral(),
										transitions={'continue': 'hsr_PoseDecision'},
										autonomy={'continue': Autonomy.Off})

			# x:387 y:317
			OperatableStateMachine.add('hsr_CreateApproach',
										hsr_CreateApproach(offset=0.3),
										transitions={'continue': 'hsr_ViewMarker', 'failed': 'hsr_MoveNeutral'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose': 'selected_pose_grasp', 'selected_pose_approach': 'selected_pose_approach', 'selected_pose_grasp': 'selected_pose_grasp'})

			# x:640 y:233
			OperatableStateMachine.add('hsr_Move2Center',
										hsr_Move2Center(before='map', after='base_footprint'),
										transitions={'continue': 'hsr_CollisionBox', 'failed': 'hsr_MoveNeutral'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_pose': 'selected_pose_grasp'})


		# x:30 y:478, x:130 y:478
		_sm_check_place_1 = OperatableStateMachine(outcomes=['yes', 'no'], input_keys=['training'])

		with _sm_check_place_1:
			# x:30 y:59
			OperatableStateMachine.add('tts_check_place',
										hsr_TtsInputKey(delay=1.5),
										transitions={'completed': 'hsr_SpeechRecognition_yes_no'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'training'})

			# x:180 y:55
			OperatableStateMachine.add('hsr_SpeechRecognition_yes_no',
										hsr_SpeechRecognition(topic="/julius/speech2text/en"),
										transitions={'recognition': 'hsr_YesNoAsk'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'yes_no'})

			# x:430 y:55
			OperatableStateMachine.add('hsr_YesNoAsk',
										hsr_YesNoAsk(),
										transitions={'yes': 'yes', 'no': 'no'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off},
										remapping={'recognition': 'yes_no'})


		# x:30 y:365, x:130 y:365
		_sm_give_2 = OperatableStateMachine(outcomes=['continue', 'failed'])

		with _sm_give_2:
			# x:30 y:44
			OperatableStateMachine.add('hsr_MoveNeutral',
										hsr_MoveNeutral(),
										transitions={'continue': 'tts_TakeThis'},
										autonomy={'continue': Autonomy.Off})

			# x:172 y:41
			OperatableStateMachine.add('tts_TakeThis',
										hsr_TtsInputParameter(text="Please take this.", language="en", delay=0.1),
										transitions={'completed': 'OpenHand'},
										autonomy={'completed': Autonomy.Off})

			# x:327 y:40
			OperatableStateMachine.add('OpenHand',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})


		# x:385 y:72
		_sm_spcolearning_3 = OperatableStateMachine(outcomes=['continue'], input_keys=['training'])

		with _sm_spcolearning_3:
			# x:51 y:46
			OperatableStateMachine.add('hsr_SpcofLearn',
										hsr_SpcofLearn(),
										transitions={'completed': 'tts_Spco_Result'},
										autonomy={'completed': Autonomy.Off},
										remapping={'request': 'training', 'spcof_text': 'spcof_text'})

			# x:214 y:54
			OperatableStateMachine.add('tts_Spco_Result',
										hsr_TtsInputKey(delay=1.5),
										transitions={'completed': 'continue'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'spcof_text'})


		# x:414 y:59
		_sm_memorizestartposition_4 = OperatableStateMachine(outcomes=['completed'], output_keys=['start_position'])

		with _sm_memorizestartposition_4:
			# x:63 y:46
			OperatableStateMachine.add('hsr_MemorizeStart_Pos',
										hsr_MemorizeStartPos(),
										transitions={'continue': 'tts_Memorize_Sart_Pos'},
										autonomy={'continue': Autonomy.Off},
										remapping={'start_position': 'start_position'})

			# x:231 y:46
			OperatableStateMachine.add('tts_Memorize_Sart_Pos',
										hsr_TtsInputParameter(text="I memorized start position.", language="en", delay=2),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off})


		# x:1253 y:624
		_sm_goto_5 = OperatableStateMachine(outcomes=['completed'])

		with _sm_goto_5:
			# x:67 y:42
			OperatableStateMachine.add('hsr_AskPlace',
										hsr_AskPlace(),
										transitions={'continue': 'tts_ask_place'},
										autonomy={'continue': Autonomy.Off},
										remapping={'place_name': 'place_name', 'question': 'question', 'place_number': 'place_number'})

			# x:704 y:432
			OperatableStateMachine.add('tts_GoTo',
										hsr_TtsInputKey(delay=1.5),
										transitions={'completed': 'Move'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'move_text'})

			# x:881 y:431
			OperatableStateMachine.add('Move',
										hsr_MoveBase(),
										transitions={'succeeded': 'tts_Arrived', 'failed': 'Limit_checker'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:691 y:548
			OperatableStateMachine.add('tts_FailedMove',
										hsr_TtsInputParameter(text="I failed to make the planning. I will try again.", language="en", delay=3),
										transitions={'completed': 'SpcoMove'},
										autonomy={'completed': Autonomy.Off})

			# x:1030 y:431
			OperatableStateMachine.add('tts_Arrived',
										hsr_TtsInputKey(delay=1.5),
										transitions={'completed': 'Manipulation'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'arrive_text'})

			# x:869 y:551
			OperatableStateMachine.add('Limit_checker',
										hsr_try_limit(),
										transitions={'give_up': 'tts_GiveupMove', 'try_again': 'tts_FailedMove'},
										autonomy={'give_up': Autonomy.Off, 'try_again': Autonomy.Off})

			# x:1024 y:556
			OperatableStateMachine.add('tts_GiveupMove',
										hsr_TtsInputParameter(text="Sorry. I gave up.", language="en", delay=3),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off})

			# x:284 y:43
			OperatableStateMachine.add('tts_ask_place',
										hsr_TtsInputKey(delay=1.5),
										transitions={'completed': 'hsr_SpeechRecognition3'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'question'})

			# x:447 y:38
			OperatableStateMachine.add('hsr_SpeechRecognition3',
										hsr_SpeechRecognition(topic="/julius/speech2text/en"),
										transitions={'recognition': 'hsr_YesNoAsk'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:283 y:158
			OperatableStateMachine.add('hsr_YesNoAsk',
										hsr_YesNoAsk(),
										transitions={'yes': 'hsr_ReadTask', 'no': 'hsr_AskPlace'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:524 y:491
			OperatableStateMachine.add('SpcoMove',
										hsr_SpcofMove(),
										transitions={'move': 'tts_GoTo', 'failed': 'SpcoMove'},
										autonomy={'move': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_name': 'place_name', 'move_text': 'move_text', 'move_pose': 'move_pose', 'arrive_text': 'arrive_text'})

			# x:687 y:217
			OperatableStateMachine.add('hsr_AskObject',
										hsr_AskObject(repeat=3),
										transitions={'continue': 'tts_AskObject', 'void_object': 'SpcoMove'},
										autonomy={'continue': Autonomy.Off, 'void_object': Autonomy.Off},
										remapping={'object_list': 'target_list', 'place_number': 'place_number', 'object_name': 'object_name', 'question': 'question'})

			# x:497 y:220
			OperatableStateMachine.add('hsr_ReadTask',
										hsr_ReadTask(),
										transitions={'continue': 'hsr_AskObject', 'failed': 'hsr_ReadTask'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'place_num': 'place_number', 'target_list': 'target_list'})

			# x:834 y:221
			OperatableStateMachine.add('tts_AskObject',
										hsr_TtsInputKey(delay=1.5),
										transitions={'completed': 'hsr_SpeechRecognition4'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'question'})

			# x:994 y:209
			OperatableStateMachine.add('hsr_SpeechRecognition4',
										hsr_SpeechRecognition(topic="/julius/speech2text/en"),
										transitions={'recognition': 'hsr_YesNoAsk_2'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:700 y:327
			OperatableStateMachine.add('hsr_YesNoAsk_2',
										hsr_YesNoAsk(),
										transitions={'yes': 'SpcoMove', 'no': 'hsr_AskObject'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:1217 y:426
			OperatableStateMachine.add('Manipulation',
										_sm_manipulation_0,
										transitions={'continue': 'completed'},
										autonomy={'continue': Autonomy.Inherit},
										remapping={'object_name': 'object_name', 'move_pose': 'move_pose'})


		# x:360 y:163
		_sm_followme_6 = OperatableStateMachine(outcomes=['completed'], input_keys=['tts_text', 'follow_me'])

		with _sm_followme_6:
			# x:43 y:153
			OperatableStateMachine.add('hsr_FollowMe',
										hsr_FollowMe(),
										transitions={'continue': 'tts_Followme'},
										autonomy={'continue': Autonomy.Off},
										remapping={'request': 'follow_me'})

			# x:204 y:150
			OperatableStateMachine.add('tts_Followme',
										hsr_TtsInputKey(delay=1.5),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'tts_text'})


		# x:751 y:313
		_sm_spcoadddata_7 = OperatableStateMachine(outcomes=['completed'], input_keys=['training'])

		with _sm_spcoadddata_7:
			# x:40 y:46
			OperatableStateMachine.add('hsr_JointPoseMemo',
										hsr_JointPose(arm_lift_joint=0.4, arm_flex_joint=-3.14/3*2, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=-0.35),
										transitions={'continue': 'hsr_ImageCaptureMemo'},
										autonomy={'continue': Autonomy.Off})

			# x:676 y:43
			OperatableStateMachine.add('hsr_LabelArrangement',
										hsr_LabelArrangement(path='/root/HSR/catkin_ws/src/em_spco_formation/training_data/default/place2object.txt'),
										transitions={'continue': 'hsr_MoveNeutralMemo', 'failed': 'hsr_LabelArrangement'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'detection_result': 'detection_memo'})

			# x:450 y:42
			OperatableStateMachine.add('hsr_BoundingBox2DMemo',
										hsr_BoundingBox2D(output_topic="/bounding_box_2d_monitor"),
										transitions={'completed': 'hsr_LabelArrangement', 'failed': 'hsr_JointPoseMemo'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image_memo', 'depth_image': 'depth_image_memo', 'camera_info': 'camera_info_memo', 'detection': 'detection_memo'})

			# x:249 y:44
			OperatableStateMachine.add('hsr_ImageCaptureMemo',
										hsr_ImageCapture(rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info"),
										transitions={'completed': 'hsr_BoundingBox2DMemo', 'failed': 'hsr_JointPoseMemo'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image_memo', 'depth_image': 'depth_image_memo', 'camera_info': 'camera_info_memo'})

			# x:914 y:46
			OperatableStateMachine.add('hsr_MoveNeutralMemo',
										hsr_MoveNeutral(),
										transitions={'continue': 'hsr_SpcofAddData'},
										autonomy={'continue': Autonomy.Off})

			# x:1129 y:106
			OperatableStateMachine.add('hsr_SpcofAddData',
										hsr_SpcofAddData(),
										transitions={'completed': 'tts_SpcoAdd'},
										autonomy={'completed': Autonomy.Off},
										remapping={'request': 'training', 'spcof_text': 'spcof_text'})

			# x:962 y:224
			OperatableStateMachine.add('tts_SpcoAdd',
										hsr_TtsInputKey(delay=1.5),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'spcof_text'})


		# x:30 y:365
		_sm_dooropen_8 = OperatableStateMachine(outcomes=['completed'])

		with _sm_dooropen_8:
			# x:33 y:40
			OperatableStateMachine.add('hsr_DoorDetect',
										hsr_DoorDetect(),
										transitions={'open': 'tts_door_opened', 'close': 'hsr_DoorDetect'},
										autonomy={'open': Autonomy.Off, 'close': Autonomy.Off})

			# x:30 y:130
			OperatableStateMachine.add('tts_door_opened',
										hsr_TtsInputParameter(text="Door opened. I can start learning phase", language="en", delay=1),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off})



		with _state_machine:
			# x:49 y:128
			OperatableStateMachine.add('DoorOpen',
										_sm_dooropen_8,
										transitions={'completed': 'hsr_DummyInitialPos'},
										autonomy={'completed': Autonomy.Inherit})

			# x:368 y:130
			OperatableStateMachine.add('hsr_SpeechRecognition',
										hsr_SpeechRecognition(topic="/julius/speech2text/en"),
										transitions={'recognition': 'hsr_Go_Get_It_Learn'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:650 y:169
			OperatableStateMachine.add('tts_Retry_Recognition',
										hsr_TtsInputParameter(text="Speak again.", language="en", delay=2),
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Off})

			# x:650 y:9
			OperatableStateMachine.add('SpcoAddData',
										_sm_spcoadddata_7,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit},
										remapping={'training': 'training'})

			# x:651 y:80
			OperatableStateMachine.add('FollowMe',
										_sm_followme_6,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit},
										remapping={'tts_text': 'tts_text', 'follow_me': 'follow_me'})

			# x:939 y:261
			OperatableStateMachine.add('tts_Finish_Train_Phase',
										hsr_TtsInputParameter(text="Finish the training phase.", language="en", delay=2),
										transitions={'completed': 'tts_ReturnFinish'},
										autonomy={'completed': Autonomy.Off})

			# x:926 y:435
			OperatableStateMachine.add('hsr_SpeechRecognition_2',
										hsr_SpeechRecognition(topic="/julius/speech2text/en"),
										transitions={'recognition': 'GoTo'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:740 y:391
			OperatableStateMachine.add('GoTo',
										_sm_goto_5,
										transitions={'completed': 'tts_Retern_Start_Pos'},
										autonomy={'completed': Autonomy.Inherit})

			# x:539 y:394
			OperatableStateMachine.add('tts_Retern_Start_Pos',
										hsr_TtsInputParameter(text="I will return to the start position.", language="en", delay=1),
										transitions={'completed': 'hsr_MoveBase'},
										autonomy={'completed': Autonomy.Off})

			# x:781 y:499
			OperatableStateMachine.add('tts_next_command',
										hsr_TtsInputParameter(text="Please give me the next command", language="en", delay=1),
										transitions={'completed': 'hsr_SpeechRecognition_2'},
										autonomy={'completed': Autonomy.Off})

			# x:466 y:499
			OperatableStateMachine.add('hsr_MoveBase',
										hsr_MoveBase(),
										transitions={'succeeded': 'Give', 'failed': 'Give'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'start_position'})

			# x:893 y:129
			OperatableStateMachine.add('hsr_Go_Get_It_Learn',
										hsr_Go_Get_It_Learn(),
										transitions={'follow_me': 'FollowMe', 'retry_recognition': 'tts_Retry_Recognition', 'data': 'Check_place', 'learn': 'SpcoLearning', 'finish': 'tts_Finish_Train_Phase', 'start_position': 'MemorizeStartPosition'},
										autonomy={'follow_me': Autonomy.Off, 'retry_recognition': Autonomy.Off, 'data': Autonomy.Off, 'learn': Autonomy.Off, 'finish': Autonomy.Off, 'start_position': Autonomy.Off},
										remapping={'recognition': 'recognition', 'follow_me': 'follow_me', 'training': 'training', 'tts_text': 'tts_text'})

			# x:631 y:237
			OperatableStateMachine.add('MemorizeStartPosition',
										_sm_memorizestartposition_4,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit},
										remapping={'start_position': 'start_position'})

			# x:203 y:130
			OperatableStateMachine.add('hsr_DummyInitialPos',
										hsr_DummyInitialPos(),
										transitions={'continue': 'hsr_SpeechRecognition'},
										autonomy={'continue': Autonomy.Off},
										remapping={'position': 'start_position'})

			# x:649 y:303
			OperatableStateMachine.add('SpcoLearning',
										_sm_spcolearning_3,
										transitions={'continue': 'hsr_SpeechRecognition'},
										autonomy={'continue': Autonomy.Inherit},
										remapping={'training': 'training'})

			# x:635 y:494
			OperatableStateMachine.add('Give',
										_sm_give_2,
										transitions={'continue': 'tts_next_command', 'failed': 'tts_next_command'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:831 y:6
			OperatableStateMachine.add('Check_place',
										_sm_check_place_1,
										transitions={'yes': 'SpcoAddData', 'no': 'hsr_SpeechRecognition'},
										autonomy={'yes': Autonomy.Inherit, 'no': Autonomy.Inherit},
										remapping={'training': 'training'})

			# x:1196 y:434
			OperatableStateMachine.add('hsr_MoveBase_Start',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_SpeechRecognition_2', 'failed': 'tts_ReturnFinish'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'start_position'})

			# x:1168 y:265
			OperatableStateMachine.add('tts_ReturnFinish',
										hsr_TtsInputParameter(text="I will return to the start position.", language="en", delay=0.1),
										transitions={'completed': 'hsr_MoveBase_Start'},
										autonomy={'completed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

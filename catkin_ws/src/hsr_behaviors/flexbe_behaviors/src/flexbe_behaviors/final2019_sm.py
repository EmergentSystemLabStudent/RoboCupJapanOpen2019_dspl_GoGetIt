#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_move_to_neutral import hsr_MoveNeutral
from hsr_flexbe_states.hsr_spcof_training_state import hsr_SpcofTraining
from hsr_flexbe_states.hsr_tts_input_key_state import hsr_TtsInputKey
from hsr_flexbe_states.hsr_spcof_move_state import hsr_SpcofMove
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBase
from hsr_flexbe_states.hsr_tts_input_parameter_state import hsr_TtsInputParameter
from hsr_flexbe_states.hsr_try_limit_state import hsr_try_limit
from hsr_flexbe_states.hsr_speech_recognition_state import hsr_SpeechRecognition
from hsr_flexbe_states.hsr_final_demo_state import hsr_Final_Demo
from hsr_flexbe_states.hsr_image_capture_state import hsr_ImageCapture
from hsr_flexbe_states.hsr_bounding_box_2d_state import hsr_BoundingBox2D
from hsr_flexbe_states.hsr_grasping_point_detect_state import hsr_GraspingPointDetect
from hsr_flexbe_states.hsr_tf2_state import hsr_Tf2
from hsr_flexbe_states.hsr_veiw_detection_state import hsr_ViewDetection
from hsr_flexbe_states.hsr_joint_pose_state import hsr_JointPose
from hsr_flexbe_states.hsr_pose_decision import hsr_PoseDecision
from hsr_flexbe_states.hsr_moveit_to_pose_goal_action_state import hsr_MoveitToPoseGoalAction
from hsr_flexbe_states.hsr_pose_dummy import hsr_PoseDummy
from hsr_flexbe_states.hsr_veiw_marker import hsr_ViewMarker
from hsr_flexbe_states.hsr_gripping_object_state import hsr_GrippingObject
from hsr_flexbe_states.hsr_collision_box_state import hsr_CollisionBox
from hsr_flexbe_states.hsr_joint_pose_line_state import hsr_JointPoseLine
from hsr_flexbe_states.hsr_create_approach import hsr_CreateApproach
from hsr_flexbe_states.hsr_move2center import hsr_Move2Center
from hsr_flexbe_states.hsr_string_dummy import hsr_StringDummy
from hsr_flexbe_states.hsr_dummy_initial_pos_state import hsr_DummyInitialPos
from flexbe_states.publisher_string_state import PublisherStringState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Aug 08 2019
@author: HitoshiNakamura
'''
class Final2019SM(Behavior):
	'''
	Final2019
	'''


	def __init__(self):
		super(Final2019SM, self).__init__()
		self.name = 'Final2019'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1191 y:328, x:926 y:522
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:478
		_sm_yoshikidesk_0 = OperatableStateMachine(outcomes=['done'])

		with _sm_yoshikidesk_0:
			# x:330 y:44
			OperatableStateMachine.add('tts_YoshikisDesk',
										hsr_TtsInputParameter(text="I add new concept. Yoshiki's desk.", language="en", delay=2.0),
										transitions={'completed': 'StringYoshiki'},
										autonomy={'completed': Autonomy.Off})

			# x:197 y:41
			OperatableStateMachine.add('StringYoshiki',
										hsr_StringDummy(text="Yoshiki's desk"),
										transitions={'continue': 'hsr_Publish_Yoshiki'},
										autonomy={'continue': Autonomy.Off},
										remapping={'name': 'yoshiki'})

			# x:30 y:40
			OperatableStateMachine.add('hsr_Publish_Yoshiki',
										PublisherStringState(topic="changed_place_name"),
										transitions={'done': 'done'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'yoshiki'})


		# x:30 y:478
		_sm_cola_1 = OperatableStateMachine(outcomes=['done'])

		with _sm_cola_1:
			# x:299 y:40
			OperatableStateMachine.add('tts_Cola',
										hsr_TtsInputParameter(text="I add new concept. Cola.", language="en", delay=2.0),
										transitions={'completed': 'StringCola'},
										autonomy={'completed': Autonomy.Off})

			# x:170 y:40
			OperatableStateMachine.add('StringCola',
										hsr_StringDummy(text="Cola"),
										transitions={'continue': 'hsr_Publish_Cola'},
										autonomy={'continue': Autonomy.Off},
										remapping={'name': 'cola'})

			# x:30 y:42
			OperatableStateMachine.add('hsr_Publish_Cola',
										PublisherStringState(topic="changed_yolo_object_name"),
										transitions={'done': 'done'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'cola'})


		# x:14 y:478
		_sm_manipulation_2 = OperatableStateMachine(outcomes=['continue'], input_keys=['object_name', 'move_pose'])

		with _sm_manipulation_2:
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
		_sm_visualize_3 = OperatableStateMachine(outcomes=['failed', 'continue'])

		with _sm_visualize_3:
			# x:30 y:41
			OperatableStateMachine.add('hsr_ImageCapture',
										hsr_ImageCapture(rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info"),
										transitions={'completed': 'hsr_BoundingBox2D', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info'})

			# x:251 y:40
			OperatableStateMachine.add('hsr_BoundingBox2D',
										hsr_BoundingBox2D(output_topic="/bounding_box_2d_monitor"),
										transitions={'completed': 'hsr_GraspingPointDetect', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'detection': 'detection'})

			# x:440 y:43
			OperatableStateMachine.add('hsr_GraspingPointDetect',
										hsr_GraspingPointDetect(output_topic="/bounding_box_2d_monitor", save=True),
										transitions={'completed': 'hsr_Tf2', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'detection': 'detection', 'grasping_point': 'grasping_point'})

			# x:645 y:41
			OperatableStateMachine.add('hsr_Tf2',
										hsr_Tf2(before="head_rgbd_sensor_rgb_frame", after="map"),
										transitions={'continue': 'hsr_ViewDetection', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'before_pose': 'grasping_point', 'after_pose': 'after_pose'})

			# x:794 y:40
			OperatableStateMachine.add('hsr_ViewDetection',
										hsr_ViewDetection(),
										transitions={'continue': 'continue'},
										autonomy={'continue': Autonomy.Off},
										remapping={'object_points': 'after_pose'})


		# x:30 y:365
		_sm_goto_4 = OperatableStateMachine(outcomes=['completed'], input_keys=['move'], output_keys=['move_pose'])

		with _sm_goto_4:
			# x:43 y:31
			OperatableStateMachine.add('SpcoMoveEntrance',
										hsr_SpcofMove(),
										transitions={'move': 'tts_GoTo', 'failed': 'SpcoMoveEntrance'},
										autonomy={'move': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_name': 'move', 'move_text': 'move_text', 'move_pose': 'move_pose', 'arrive_text': 'arrive_text'})

			# x:221 y:31
			OperatableStateMachine.add('tts_GoTo',
										hsr_TtsInputKey(delay=1.5),
										transitions={'completed': 'Move'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'move_text'})

			# x:406 y:29
			OperatableStateMachine.add('Move',
										hsr_MoveBase(),
										transitions={'succeeded': 'tts_Arrived', 'failed': 'Limit_checker'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:232 y:161
			OperatableStateMachine.add('tts_FailedMove',
										hsr_TtsInputParameter(text="I failed to make the planning. I will try again.", language="en", delay=2),
										transitions={'completed': 'SpcoMoveEntrance'},
										autonomy={'completed': Autonomy.Off})

			# x:615 y:28
			OperatableStateMachine.add('tts_Arrived',
										hsr_TtsInputKey(delay=1.5),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'arrive_text'})

			# x:480 y:168
			OperatableStateMachine.add('Limit_checker',
										hsr_try_limit(),
										transitions={'give_up': 'tts_GiveupMove', 'try_again': 'tts_FailedMove'},
										autonomy={'give_up': Autonomy.Off, 'try_again': Autonomy.Off})

			# x:634 y:164
			OperatableStateMachine.add('tts_GiveupMove',
										hsr_TtsInputParameter(text="Sorry. I gave up.", language="en", delay=2),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off})


		# x:411 y:97
		_sm_learnspatialconcept_5 = OperatableStateMachine(outcomes=['completed'], input_keys=['training'])

		with _sm_learnspatialconcept_5:
			# x:50 y:85
			OperatableStateMachine.add('hsr_SpcofTraining',
										hsr_SpcofTraining(),
										transitions={'completed': 'tts_Spco_Result'},
										autonomy={'completed': Autonomy.Off},
										remapping={'request': 'training', 'spcof_text': 'spcof_text'})

			# x:239 y:81
			OperatableStateMachine.add('tts_Spco_Result',
										hsr_TtsInputKey(delay=1.5),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'spcof_text'})


		# x:30 y:365
		_sm_start_6 = OperatableStateMachine(outcomes=['completed'])

		with _sm_start_6:
			# x:30 y:43
			OperatableStateMachine.add('hsr_MoveNeutral',
										hsr_MoveNeutral(),
										transitions={'continue': 'completed'},
										autonomy={'continue': Autonomy.Off})



		with _state_machine:
			# x:97 y:133
			OperatableStateMachine.add('Start',
										_sm_start_6,
										transitions={'completed': 'hsr_DummyInitialPos'},
										autonomy={'completed': Autonomy.Inherit})

			# x:644 y:174
			OperatableStateMachine.add('LearnSpatialConcept',
										_sm_learnspatialconcept_5,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit},
										remapping={'training': 'training'})

			# x:657 y:465
			OperatableStateMachine.add('GoTo',
										_sm_goto_4,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit},
										remapping={'move': 'move', 'move_pose': 'move_pose'})

			# x:428 y:226
			OperatableStateMachine.add('hsr_SpeechRecognition',
										hsr_SpeechRecognition(topic="/julius/speech2text/en"),
										transitions={'recognition': 'hsr_FinalDemo'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:658 y:87
			OperatableStateMachine.add('tts_RetryRecognition',
										hsr_TtsInputParameter(text="Could you speak again?", language="en", delay=2),
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Off})

			# x:923 y:224
			OperatableStateMachine.add('hsr_FinalDemo',
										hsr_Final_Demo(),
										transitions={'follow_me': 'tts_RetryRecognition', 'retry_recognition': 'tts_RetryRecognition', 'data': 'LearnSpatialConcept', 'learn': 'LearnSpatialConcept', 'goto': 'GoTo', 'bring_me': 'tts_Where', 'change_cola': 'Cola', 'change_desk': 'YoshikiDesk', 'take_cola': 'Visualize', 'take_cola_again': 'Dummy_Bottle'},
										autonomy={'follow_me': Autonomy.Off, 'retry_recognition': Autonomy.Off, 'data': Autonomy.Off, 'learn': Autonomy.Off, 'goto': Autonomy.Off, 'bring_me': Autonomy.Off, 'change_cola': Autonomy.Off, 'change_desk': Autonomy.Off, 'take_cola': Autonomy.Off, 'take_cola_again': Autonomy.Off},
										remapping={'recognition': 'recognition', 'follow_me': 'follow_me', 'training': 'training', 'tts_text': 'tts_text', 'move': 'move'})

			# x:653 y:292
			OperatableStateMachine.add('tts_Where',
										hsr_TtsInputParameter(text="Where is yoshiki's desk?", language="en", delay=2),
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Off})

			# x:644 y:383
			OperatableStateMachine.add('tts_Nocola',
										hsr_TtsInputParameter(text="I can't find the cola", language="en", delay=2),
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Off})

			# x:942 y:378
			OperatableStateMachine.add('Visualize',
										_sm_visualize_3,
										transitions={'failed': 'failed', 'continue': 'tts_Nocola'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit})

			# x:657 y:572
			OperatableStateMachine.add('Manipulation',
										_sm_manipulation_2,
										transitions={'continue': 'hsr_SpeechRecognition'},
										autonomy={'continue': Autonomy.Inherit},
										remapping={'object_name': 'name', 'move_pose': 'move_pose'})

			# x:801 y:573
			OperatableStateMachine.add('Dummy_Bottle',
										hsr_StringDummy(text="bottle"),
										transitions={'continue': 'Manipulation'},
										autonomy={'continue': Autonomy.Off},
										remapping={'name': 'name'})

			# x:235 y:229
			OperatableStateMachine.add('hsr_DummyInitialPos',
										hsr_DummyInitialPos(),
										transitions={'continue': 'hsr_SpeechRecognition'},
										autonomy={'continue': Autonomy.Off},
										remapping={'position': 'move_pose'})

			# x:656 y:655
			OperatableStateMachine.add('Cola',
										_sm_cola_1,
										transitions={'done': 'hsr_SpeechRecognition'},
										autonomy={'done': Autonomy.Inherit})

			# x:656 y:731
			OperatableStateMachine.add('YoshikiDesk',
										_sm_yoshikidesk_0,
										transitions={'done': 'hsr_SpeechRecognition'},
										autonomy={'done': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

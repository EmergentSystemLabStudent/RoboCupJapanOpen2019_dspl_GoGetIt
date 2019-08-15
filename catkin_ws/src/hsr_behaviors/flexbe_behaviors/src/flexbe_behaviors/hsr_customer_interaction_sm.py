#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_reset_octomap import hsr_ResetOctomap
from hsr_flexbe_states.hsr_customer_interaction_state import hsr_CustomerInteraction
from hsr_flexbe_states.hsr_follow_me_state import hsr_FollowMe
from hsr_flexbe_states.hsr_tts_input_key_state import hsr_TtsInputKey
from hsr_flexbe_states.hsr_spcof_move_state import hsr_SpcofMove
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBase
from hsr_flexbe_states.hsr_tts_input_parameter_state import hsr_TtsInputParameter
from flexbe_states.wait_state import WaitState
from hsr_flexbe_states.hsr_string_dummy import hsr_StringDummy
from hsr_flexbe_states.hsr_rosbridge_event import hsr_RosBridgeEvent
from hsr_flexbe_states.hsr_spcof_training_state import hsr_SpcofTraining
from hsr_flexbe_states.hsr_TidyUpDummy import hsr_TidyUpDummy
from hsr_flexbe_states.hsr_joint_pose_state import hsr_JointPose
from hsr_flexbe_states.hsr_speech_recognition_state import hsr_SpeechRecognition
from hsr_flexbe_states.hsr_clear_bbox_octomap import hsr_ClearBBoxOctomap
from hsr_flexbe_states.hsr_test_bbox import hsr_TestBBox
from hsr_flexbe_states.hsr_planning_scene_control import hsr_PlanningSceneControl
from hsr_flexbe_states.hsr_bounding_box_2d_state import hsr_BoundingBox2D
from hsr_flexbe_states.hsr_grasping_point_detect_state import hsr_GraspingPointDetect
from hsr_flexbe_states.hsr_tf2_state import hsr_Tf2
from hsr_flexbe_states.hsr_pose_decision_conveni import hsr_PoseDecisionConveni
from hsr_flexbe_states.hsr_gripping_object_state import hsr_GrippingObject
from hsr_flexbe_states.hsr_moveit_to_pose_goal_action_state import hsr_MoveitToPoseGoalAction
from hsr_flexbe_states.hsr_omni_base_state import hsr_OmniBase
from hsr_flexbe_states.hsr_veiw_marker import hsr_ViewMarker
from hsr_flexbe_states.hsr_image_capture_state import hsr_ImageCapture
from hsr_flexbe_states.hsr_get_hand_pose import hsr_GetHandPose
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Oct 10 2018
@author: tabuchi
'''
class hsr_customer_interactionSM(Behavior):
	'''
	Customer Interaction state
	'''


	def __init__(self):
		super(hsr_customer_interactionSM, self).__init__()
		self.name = 'hsr_customer_interaction'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:20 y:878, x:1090 y:620
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]

		# x:494 y:31
		_sm_future1_0 = OperatableStateMachine(outcomes=['completed'])

		with _sm_future1_0:
			# x:83 y:26
			OperatableStateMachine.add('event_display_cool',
										hsr_RosBridgeEvent(Event_ID="28"),
										transitions={'completed': 'tts_Future1', 'failed': 'event_display_cool'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:272 y:28
			OperatableStateMachine.add('tts_Future1',
										hsr_TtsInputParameter(text="I can show you the future! Would you like to see it? Please run the future convenience store app on your smartphone and tell me when you are ready.", language="en"),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off})


		# x:1036 y:155
		_sm_future3_1 = OperatableStateMachine(outcomes=['completed'])

		with _sm_future3_1:
			# x:46 y:42
			OperatableStateMachine.add('tts_future3',
										hsr_TtsInputParameter(text="Sure! I will update the future convenience store app on your smartphone with a new digital point card. Please wait a moment.", language="en"),
										transitions={'completed': 'hsr_put_product'},
										autonomy={'completed': Autonomy.Off})

			# x:493 y:39
			OperatableStateMachine.add('event_display_product',
										hsr_RosBridgeEvent(Event_ID="27"),
										transitions={'completed': 'wait_10sec', 'failed': 'event_display_product'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:807 y:42
			OperatableStateMachine.add('tts_future4',
										hsr_TtsInputParameter(text="Done! You now have a new digital point card on your smartphone.", language="en"),
										transitions={'completed': 'hsr_neutral'},
										autonomy={'completed': Autonomy.Off})

			# x:1141 y:137
			OperatableStateMachine.add('tts_future5',
										hsr_TtsInputParameter(text="Thank you so much for visiting us today. I hoped you liked our future convenience store!", language="en"),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off})

			# x:341 y:41
			OperatableStateMachine.add('hsr_GetHandPose',
										hsr_GetHandPose(),
										transitions={'completed': 'event_display_product', 'failed': 'hsr_GetHandPose'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:678 y:37
			OperatableStateMachine.add('wait_10sec',
										WaitState(wait_time=10),
										transitions={'done': 'tts_future4'},
										autonomy={'done': Autonomy.Off})

			# x:210 y:41
			OperatableStateMachine.add('hsr_put_product',
										hsr_JointPose(arm_lift_joint=0.3, arm_flex_joint=-3.14/4, arm_roll_joint=0.0, wrist_flex_joint=-3.14/4, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.3),
										transitions={'continue': 'hsr_GetHandPose'},
										autonomy={'continue': Autonomy.Off})

			# x:992 y:41
			OperatableStateMachine.add('hsr_neutral',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'event_display_happy'},
										autonomy={'continue': Autonomy.Off})

			# x:1140 y:41
			OperatableStateMachine.add('event_display_happy',
										hsr_RosBridgeEvent(Event_ID="20"),
										transitions={'completed': 'tts_future5', 'failed': 'event_display_happy'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})


		# x:1364 y:555, x:1916 y:209, x:1069 y:241, x:972 y:475
		_sm_newest_product_2 = OperatableStateMachine(outcomes=['failed', 'planning_failed', 'control_failed', 'continue'], input_keys=['move', 'object'], output_keys=['selected_object_status'])

		with _sm_newest_product_2:
			# x:58 y:28
			OperatableStateMachine.add('hsr_SpcofMove2',
										hsr_SpcofMove(),
										transitions={'move': 'hsr_JointPose_xtion', 'failed': 'hsr_SpcofMove2'},
										autonomy={'move': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_name': 'move', 'move_text': 'move_text', 'move_pose': 'move_pose', 'arrive_text': 'arrive_text'})

			# x:1169 y:26
			OperatableStateMachine.add('hsr_BoundingBox2D',
										hsr_BoundingBox2D(output_topic='/bounding_box_2d_monitor'),
										transitions={'completed': 'hsr_GraspingPointDetect', 'failed': 'hsr_BoundingBox2D'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'detection': 'detection', 'grasping_point': 'grasping_point'})

			# x:1348 y:24
			OperatableStateMachine.add('hsr_GraspingPointDetect',
										hsr_GraspingPointDetect(output_topic="/bounding_box_2d_monitor", save=True),
										transitions={'completed': 'hsr_Tf2', 'failed': 'hsr_GraspingPointDetect'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'detection': 'detection', 'grasping_point': 'grasping_point'})

			# x:1552 y:24
			OperatableStateMachine.add('hsr_Tf2',
										hsr_Tf2(before='head_rgbd_sensor_rgb_frame', after='map'),
										transitions={'continue': 'hsr_JointPose_arm', 'failed': 'hsr_Tf2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'before_pose': 'grasping_point', 'after_pose': 'after_pose'})

			# x:1318 y:130
			OperatableStateMachine.add('hsr_PoseDecisionConveni',
										hsr_PoseDecisionConveni(offset=None),
										transitions={'continue': 'hsr_ViewGraspPoint', 'plan_failed': 'tts_FailPlan', 'detect_failed': 'tts_RecognitionFailed'},
										autonomy={'continue': Autonomy.Off, 'plan_failed': Autonomy.Off, 'detect_failed': Autonomy.Off},
										remapping={'target_poses': 'after_pose', 'target_object_names': 'object', 'failed_objects': 'object', 'selected_pose_approach': 'selected_pose_approach', 'selected_pose_grasp': 'selected_pose_grasp', 'selected_object_name': 'selected_object_name', 'selected_object_status': 'selected_object_status'})

			# x:766 y:125
			OperatableStateMachine.add('hsr_CloseHand',
										hsr_GrippingObject(grasp_force=0.7, mode=True),
										transitions={'continue': 'hsr_MoveBase_back', 'failed': 'hsr_MoveBase_failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'selected_object_status'})

			# x:935 y:127
			OperatableStateMachine.add('hsr_MoveitToPoseGoalAction',
										hsr_MoveitToPoseGoalAction(move_group='whole_body', action_topic='/move_group', tolerance=0.001, orien_tolerance=True),
										transitions={'reached': 'hsr_CloseHand', 'planning_failed': 'tts_FailPlan', 'control_failed': 'control_failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'pose_goal': 'selected_pose_grasp', 'move_status': 'selected_object_status'})

			# x:426 y:28
			OperatableStateMachine.add('hsr_OpenHand',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_ResumeOctomap', 'failed': 'hsr_OpenHand'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:241 y:29
			OperatableStateMachine.add('hsr_JointPose_xtion',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_OpenHand'},
										autonomy={'continue': Autonomy.Off})

			# x:1712 y:20
			OperatableStateMachine.add('hsr_JointPose_arm',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_ClearBBoxOctomap'},
										autonomy={'continue': Autonomy.Off})

			# x:574 y:121
			OperatableStateMachine.add('hsr_MoveBase_back',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_JointPose_initial1', 'failed': 'hsr_MoveBase_back'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:57 y:236
			OperatableStateMachine.add('hsr_JointPose_put',
										hsr_JointPose(arm_lift_joint=0.3, arm_flex_joint=-3.14/4, arm_roll_joint=0.0, wrist_flex_joint=-3.14/4, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.3),
										transitions={'continue': 'tts_HereYouAre'},
										autonomy={'continue': Autonomy.Off})

			# x:56 y:326
			OperatableStateMachine.add('tts_HereYouAre',
										hsr_TtsInputParameter(text="Here you are.", language="en"),
										transitions={'completed': 'event_display_happy'},
										autonomy={'completed': Autonomy.Off})

			# x:562 y:326
			OperatableStateMachine.add('hsr_HandOver',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_JointPose_initial2', 'failed': 'hsr_HandOver'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:724 y:328
			OperatableStateMachine.add('hsr_JointPose_initial2',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'tts_ProductDelivered'},
										autonomy={'continue': Autonomy.Off})

			# x:59 y:127
			OperatableStateMachine.add('hsr_OmniBase',
										hsr_OmniBase(x=0.0, y=0.0, yaw=3.14/2, time_out=30.0),
										transitions={'finish': 'hsr_JointPose_put'},
										autonomy={'finish': Autonomy.Off})

			# x:389 y:124
			OperatableStateMachine.add('hsr_JointPose_initial1',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_PauseOctomap'},
										autonomy={'continue': Autonomy.Off})

			# x:608 y:27
			OperatableStateMachine.add('hsr_ResumeOctomap',
										hsr_ResetOctomap(octomap_topic="/octomap_server/reset"),
										transitions={'completed': 'hsr_PlanningSceneStart', 'failed': 'hsr_ResumeOctomap'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:229 y:126
			OperatableStateMachine.add('hsr_PauseOctomap',
										hsr_ResetOctomap(octomap_topic="/octomap_server/reset"),
										transitions={'completed': 'hsr_OmniBase', 'failed': 'hsr_PauseOctomap'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:794 y:26
			OperatableStateMachine.add('hsr_PlanningSceneStart',
										hsr_PlanningSceneControl(update_sw=True, wait_time=0.0),
										transitions={'completed': 'hsr_ImageCapture', 'failed': 'hsr_PlanningSceneStart'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1691 y:126
			OperatableStateMachine.add('hsr_ClearBBoxOctomap',
										hsr_ClearBBoxOctomap(topic_name="octomap_server/clear_bbx", bbox_param_xy=0.1, bbox_param_z=0.17),
										transitions={'completed': 'hsr_PlanningSceneStop', 'failed': 'hsr_ClearBBoxOctomap'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_point': 'after_pose'})

			# x:1517 y:129
			OperatableStateMachine.add('hsr_PlanningSceneStop',
										hsr_PlanningSceneControl(update_sw=False, wait_time=0.0),
										transitions={'completed': 'hsr_PoseDecisionConveni', 'failed': 'hsr_PlanningSceneStop'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1264 y:337
			OperatableStateMachine.add('tts_FailPlan',
										hsr_TtsInputParameter(text="I failed to plan a trajectory, but I will try again.", language="en"),
										transitions={'completed': 'failed'},
										autonomy={'completed': Autonomy.Off})

			# x:1150 y:128
			OperatableStateMachine.add('hsr_ViewGraspPoint',
										hsr_ViewMarker(),
										transitions={'continue': 'hsr_MoveitToPoseGoalAction', 'failed': 'hsr_ViewGraspPoint'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'view_pose': 'selected_pose_grasp'})

			# x:998 y:25
			OperatableStateMachine.add('hsr_ImageCapture',
										hsr_ImageCapture(rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info"),
										transitions={'completed': 'hsr_BoundingBox2D', 'failed': 'hsr_ImageCapture'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info'})

			# x:902 y:327
			OperatableStateMachine.add('tts_ProductDelivered',
										hsr_TtsInputParameter(text="Is there something else I can do for you?", language="en"),
										transitions={'completed': 'continue'},
										autonomy={'completed': Autonomy.Off})

			# x:1586 y:310
			OperatableStateMachine.add('tts_RecognitionFailed',
										hsr_TtsInputParameter(text="I cannot see the item, but I will try harder!", language="en"),
										transitions={'completed': 'failed'},
										autonomy={'completed': Autonomy.Off})

			# x:404 y:326
			OperatableStateMachine.add('WaitState_2sec',
										WaitState(wait_time=2.0),
										transitions={'done': 'hsr_HandOver'},
										autonomy={'done': Autonomy.Off})

			# x:231 y:327
			OperatableStateMachine.add('event_display_happy',
										hsr_RosBridgeEvent(Event_ID="20"),
										transitions={'completed': 'WaitState_2sec', 'failed': 'event_display_happy'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:758 y:224
			OperatableStateMachine.add('hsr_MoveBase_failed',
										hsr_MoveBase(),
										transitions={'succeeded': 'failed', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})


		# x:475 y:26
		_sm_future2_3 = OperatableStateMachine(outcomes=['completed'])

		with _sm_future2_3:
			# x:64 y:26
			OperatableStateMachine.add('event_display_shelf',
										hsr_RosBridgeEvent(Event_ID="26"),
										transitions={'completed': 'tts_Future2', 'failed': 'event_display_shelf'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:264 y:26
			OperatableStateMachine.add('tts_Future2',
										hsr_TtsInputParameter(text="Me too! Please look at me with your smartphone to see how I can augment the reality with virtual products and services. Would you like to join or point-based reward club? Or offer a digital gift card? Or purchase digital music, movies, books and games? I can send them directly to your smartphone.", language="en"),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off})


		# x:30 y:514
		_sm_move_to_newest_product_4 = OperatableStateMachine(outcomes=['completed'], input_keys=['move'])

		with _sm_move_to_newest_product_4:
			# x:30 y:43
			OperatableStateMachine.add('hsr_SpcofMove_newproduct',
										hsr_SpcofMove(),
										transitions={'move': 'tts_NewProduct_go', 'failed': 'hsr_SpcofMove_newproduct'},
										autonomy={'move': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_name': 'move', 'move_text': 'move_text', 'move_pose': 'move_pose', 'arrive_text': 'arrive_text'})

			# x:256 y:43
			OperatableStateMachine.add('tts_NewProduct_go',
										hsr_TtsInputParameter(text="Yes! Please follow me.", language="en"),
										transitions={'completed': 'MoveBase_NewProduct'},
										autonomy={'completed': Autonomy.Off})

			# x:425 y:41
			OperatableStateMachine.add('MoveBase_NewProduct',
										hsr_MoveBase(),
										transitions={'succeeded': 'tts_NewProduct_arrive', 'failed': 'tts_NewProduct_failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:256 y:122
			OperatableStateMachine.add('tts_NewProduct_failed',
										hsr_TtsInputParameter(text="I could not find a route, but I will try again.", language="en"),
										transitions={'completed': 'hsr_SpcofMove_newproduct'},
										autonomy={'completed': Autonomy.Off})

			# x:611 y:40
			OperatableStateMachine.add('tts_NewProduct_arrive',
										hsr_TtsInputParameter(text="Here is what you asked me, the newest product. Please wait a moment, I will try to take it for you.", language="en"),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off})


		# x:844 y:36
		_sm_grasp_fail_talk_5 = OperatableStateMachine(outcomes=['continue'])

		with _sm_grasp_fail_talk_5:
			# x:73 y:26
			OperatableStateMachine.add('tts_GraspFail',
										hsr_TtsInputParameter(text="I am sorry, I cannot grasp it. Please help yourself.", language="en"),
										transitions={'completed': 'hsr_JointPose_initial3'},
										autonomy={'completed': Autonomy.Off})

			# x:281 y:27
			OperatableStateMachine.add('hsr_JointPose_initial3',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'WaitState_3sec'},
										autonomy={'continue': Autonomy.Off})

			# x:470 y:26
			OperatableStateMachine.add('WaitState_3sec',
										WaitState(wait_time=3),
										transitions={'done': 'tts_GraspFailed'},
										autonomy={'done': Autonomy.Off})

			# x:633 y:25
			OperatableStateMachine.add('tts_GraspFailed',
										hsr_TtsInputParameter(text="Is there something else I can do for you?", language="en"),
										transitions={'completed': 'continue'},
										autonomy={'completed': Autonomy.Off})


		# x:648 y:32
		_sm_spatial_concept_6 = OperatableStateMachine(outcomes=['completed'], input_keys=['training'])

		with _sm_spatial_concept_6:
			# x:34 y:25
			OperatableStateMachine.add('event_display_happy',
										hsr_RosBridgeEvent(Event_ID="20"),
										transitions={'completed': 'hsr_SpcofTraining', 'failed': 'event_display_happy'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:333 y:26
			OperatableStateMachine.add('tts_SpcofTraining',
										hsr_TtsInputKey(),
										transitions={'completed': 'event_display_think'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'spcof_text'})

			# x:34 y:195
			OperatableStateMachine.add('tts_SpcofLearning',
										hsr_TtsInputParameter(text=" I am processing the data in my memory to learn new concepts. Please wait a moment.", language="en"),
										transitions={'completed': 'event_display_happy'},
										autonomy={'completed': Autonomy.Off})

			# x:474 y:27
			OperatableStateMachine.add('event_display_think',
										hsr_RosBridgeEvent(Event_ID="29"),
										transitions={'completed': 'completed', 'failed': 'event_display_think'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:185 y:25
			OperatableStateMachine.add('hsr_SpcofTraining',
										hsr_SpcofTraining(),
										transitions={'completed': 'tts_SpcofTraining'},
										autonomy={'completed': Autonomy.Off},
										remapping={'request': 'training', 'spcof_text': 'spcof_text'})


		# x:1620 y:29
		_sm_toilet_7 = OperatableStateMachine(outcomes=['completed'], input_keys=['move'])

		with _sm_toilet_7:
			# x:54 y:26
			OperatableStateMachine.add('hsr_SpcofMove_Toilet',
										hsr_SpcofMove(),
										transitions={'move': 'tts_Toilet_go', 'failed': 'hsr_SpcofMove_Toilet'},
										autonomy={'move': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_name': 'move', 'move_text': 'move_text', 'move_pose': 'move_pose', 'arrive_text': 'arrive_text'})

			# x:248 y:26
			OperatableStateMachine.add('tts_Toilet_go',
										hsr_TtsInputParameter(text="Yes! Please follow me.", language="en"),
										transitions={'completed': 'MoveBase_Toilet'},
										autonomy={'completed': Autonomy.Off})

			# x:431 y:25
			OperatableStateMachine.add('MoveBase_Toilet',
										hsr_MoveBase(),
										transitions={'succeeded': 'tts_Toilet_arrive', 'failed': 'tts_Toilet_failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:585 y:24
			OperatableStateMachine.add('tts_Toilet_arrive',
										hsr_TtsInputParameter(text="Here is what you asked me, the toilet. Please take your time. I will go back to the entrance.", language="en"),
										transitions={'completed': 'hsr_StringDummy'},
										autonomy={'completed': Autonomy.Off})

			# x:250 y:126
			OperatableStateMachine.add('tts_Toilet_failed',
										hsr_TtsInputParameter(text="I could not find a route, but I will try again.", language="en"),
										transitions={'completed': 'hsr_SpcofMove_Toilet'},
										autonomy={'completed': Autonomy.Off})

			# x:1204 y:25
			OperatableStateMachine.add('hsr_SpcofMove_Entrance2',
										hsr_SpcofMove(),
										transitions={'move': 'MoveBase_Entrance2', 'failed': 'hsr_SpcofMove_Entrance2'},
										autonomy={'move': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_name': 'name', 'move_text': 'move_text', 'move_pose': 'move_pose', 'arrive_text': 'arrive_text'})

			# x:1417 y:24
			OperatableStateMachine.add('MoveBase_Entrance2',
										hsr_MoveBase(),
										transitions={'succeeded': 'completed', 'failed': 'MoveBase_Entrance2'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:752 y:22
			OperatableStateMachine.add('hsr_StringDummy',
										hsr_StringDummy(text="entrance"),
										transitions={'continue': 'event_display_happy'},
										autonomy={'continue': Autonomy.Off},
										remapping={'name': 'name'})

			# x:936 y:22
			OperatableStateMachine.add('event_display_happy',
										hsr_RosBridgeEvent(Event_ID="20"),
										transitions={'completed': 'hsr_SpcofMove_Entrance2', 'failed': 'event_display_happy'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})


		# x:791 y:31
		_sm_wait_8 = OperatableStateMachine(outcomes=['completed'])

		with _sm_wait_8:
			# x:40 y:26
			OperatableStateMachine.add('tts_Wait',
										hsr_TtsInputParameter(text="I am starting my duty!", language="en"),
										transitions={'completed': 'WaitState'},
										autonomy={'completed': Autonomy.Off})

			# x:225 y:25
			OperatableStateMachine.add('WaitState',
										WaitState(wait_time=10),
										transitions={'done': 'tts_Welcome'},
										autonomy={'done': Autonomy.Off})

			# x:577 y:24
			OperatableStateMachine.add('tts_Welcome',
										hsr_TtsInputParameter(text="Welcome to the future convenience store. Please ask me anything.", language="en"),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off})


		# x:849 y:33
		_sm_entrance_9 = OperatableStateMachine(outcomes=['completed'], input_keys=['move'])

		with _sm_entrance_9:
			# x:46 y:26
			OperatableStateMachine.add('hsr_SpcofMove_Entrance',
										hsr_SpcofMove(),
										transitions={'move': 'tts_Entrance_go', 'failed': 'hsr_SpcofMove_Entrance'},
										autonomy={'move': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_name': 'move', 'move_text': 'move_text', 'move_pose': 'move_pose', 'arrive_text': 'arrive_text'})

			# x:275 y:26
			OperatableStateMachine.add('tts_Entrance_go',
										hsr_TtsInputKey(),
										transitions={'completed': 'MoveBase_Entrance'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'move_text'})

			# x:442 y:24
			OperatableStateMachine.add('MoveBase_Entrance',
										hsr_MoveBase(),
										transitions={'succeeded': 'tts_Entrance_arrive', 'failed': 'tts_Entrance_failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:638 y:24
			OperatableStateMachine.add('tts_Entrance_arrive',
										hsr_TtsInputKey(),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'arrive_text'})

			# x:267 y:167
			OperatableStateMachine.add('tts_Entrance_failed',
										hsr_TtsInputParameter(text="I could not find a route, but I will try again.", language="en"),
										transitions={'completed': 'hsr_SpcofMove_Entrance'},
										autonomy={'completed': Autonomy.Off})


		# x:351 y:30
		_sm_follow_me_10 = OperatableStateMachine(outcomes=['completed'], input_keys=['tts_text', 'follow_me'])

		with _sm_follow_me_10:
			# x:43 y:153
			OperatableStateMachine.add('hsr_FollowMe',
										hsr_FollowMe(),
										transitions={'continue': 'tts_Followme'},
										autonomy={'continue': Autonomy.Off},
										remapping={'request': 'follow_me'})

			# x:204 y:150
			OperatableStateMachine.add('tts_Followme',
										hsr_TtsInputKey(),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'tts_text'})



		with _state_machine:
			# x:36 y:104
			OperatableStateMachine.add('hsr_PauseOctomap',
										hsr_ResetOctomap(octomap_topic="/octomap_server/reset"),
										transitions={'completed': 'hsr_ClearCenterPoint', 'failed': 'hsr_PauseOctomap'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:827 y:390
			OperatableStateMachine.add('hsr_CustomerInteraction',
										hsr_CustomerInteraction(),
										transitions={'follow_me': 'Follow Me', 'data': 'Spatial Concept', 'learn': 'Spatial Concept', 'entrance': 'Entrance', 'wait': 'Wait', 'newest_product': 'Move to Newest product', 'retry_newest_product': 'Newest Product', 'toilet': 'Toilet', 'future1': 'future1', 'future2': 'future2', 'future3': 'future3', 'grasp_fail_talk': 'Grasp fail talk', 'success_grasp': 'hsr_SpeechRecognition', 'retry_recognition': 'hsr_SpeechRecognition'},
										autonomy={'follow_me': Autonomy.Off, 'data': Autonomy.Off, 'learn': Autonomy.Off, 'entrance': Autonomy.Off, 'wait': Autonomy.Off, 'newest_product': Autonomy.Off, 'retry_newest_product': Autonomy.Off, 'toilet': Autonomy.Off, 'future1': Autonomy.Off, 'future2': Autonomy.Off, 'future3': Autonomy.Off, 'grasp_fail_talk': Autonomy.Off, 'success_grasp': Autonomy.Off, 'retry_recognition': Autonomy.Off},
										remapping={'recognition': 'recognition', 'grasp_status': 'selected_object_status', 'follow_me': 'follow_me', 'training': 'training', 'move': 'move', 'object': 'object', 'tts_text': 'tts_text'})

			# x:503 y:3
			OperatableStateMachine.add('Follow Me',
										_sm_follow_me_10,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit},
										remapping={'tts_text': 'tts_text', 'follow_me': 'follow_me'})

			# x:506 y:211
			OperatableStateMachine.add('Entrance',
										_sm_entrance_9,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit},
										remapping={'move': 'move'})

			# x:506 y:308
			OperatableStateMachine.add('Wait',
										_sm_wait_8,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit})

			# x:509 y:638
			OperatableStateMachine.add('Toilet',
										_sm_toilet_7,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit},
										remapping={'move': 'move'})

			# x:497 y:106
			OperatableStateMachine.add('Spatial Concept',
										_sm_spatial_concept_6,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit},
										remapping={'training': 'training'})

			# x:512 y:503
			OperatableStateMachine.add('hsr_Dummy',
										hsr_TidyUpDummy(),
										transitions={'continue': 'hsr_CustomerInteraction'},
										autonomy={'continue': Autonomy.Off},
										remapping={'dummy1': 'selected_object_status', 'dummy2': 'dummy2'})

			# x:511 y:568
			OperatableStateMachine.add('Grasp fail talk',
										_sm_grasp_fail_talk_5,
										transitions={'continue': 'hsr_SpeechRecognition'},
										autonomy={'continue': Autonomy.Inherit})

			# x:146 y:391
			OperatableStateMachine.add('hsr_SpeechRecognition',
										hsr_SpeechRecognition(topic="/julius/speech2text/en"),
										transitions={'recognition': 'hsr_Dummy'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:35 y:249
			OperatableStateMachine.add('hsr_ClearBBoxOctomap',
										hsr_ClearBBoxOctomap(topic_name="octomap_server/clear_bbx", bbox_param_xy=7.0, bbox_param_z=7.0),
										transitions={'completed': 'hsr_PlanningSceneStop', 'failed': 'hsr_ClearBBoxOctomap'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_point': 'object_point'})

			# x:35 y:175
			OperatableStateMachine.add('hsr_ClearCenterPoint',
										hsr_TestBBox(),
										transitions={'completed': 'hsr_ClearBBoxOctomap', 'failed': 'hsr_ClearCenterPoint'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_point': 'object_point'})

			# x:32 y:317
			OperatableStateMachine.add('hsr_PlanningSceneStop',
										hsr_PlanningSceneControl(update_sw=False, wait_time=0.0),
										transitions={'completed': 'hsr_SpeechRecognition', 'failed': 'hsr_PlanningSceneStop'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1130 y:293
			OperatableStateMachine.add('Move to Newest product',
										_sm_move_to_newest_product_4,
										transitions={'completed': 'Newest Product'},
										autonomy={'completed': Autonomy.Inherit},
										remapping={'move': 'move'})

			# x:508 y:786
			OperatableStateMachine.add('future2',
										_sm_future2_3,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit})

			# x:1152 y:477
			OperatableStateMachine.add('Newest Product',
										_sm_newest_product_2,
										transitions={'failed': 'hsr_CustomerInteraction', 'planning_failed': 'hsr_CustomerInteraction', 'control_failed': 'failed', 'continue': 'hsr_CustomerInteraction'},
										autonomy={'failed': Autonomy.Inherit, 'planning_failed': Autonomy.Inherit, 'control_failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'move': 'move', 'object': 'object', 'selected_object_status': 'selected_object_status'})

			# x:506 y:863
			OperatableStateMachine.add('future3',
										_sm_future3_1,
										transitions={'completed': 'finished'},
										autonomy={'completed': Autonomy.Inherit})

			# x:508 y:710
			OperatableStateMachine.add('future1',
										_sm_future1_0,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]

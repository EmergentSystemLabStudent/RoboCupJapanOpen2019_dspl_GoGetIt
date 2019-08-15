#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_tts_input_parameter_state import hsr_TtsInputParameter
from hsr_flexbe_states.hsr_gripping_object_state import hsr_GrippingObject
from hsr_flexbe_states.hsr_move_base_client import hsr_MoveBaseClient
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBase
from hsr_flexbe_states.hsr_joint_pose_state import hsr_JointPose
from hsr_flexbe_states.hsr_image_capture_state import hsr_ImageCapture
from hsr_flexbe_states.hsr_bounding_box_2d_state import hsr_BoundingBox2D
from hsr_flexbe_states.hsr_grasping_point_detect_state import hsr_GraspingPointDetect
from hsr_flexbe_states.hsr_tf2_state import hsr_Tf2
from hsr_flexbe_states.hsr_pose_decision_conveni import hsr_PoseDecisionConveni
from hsr_flexbe_states.hsr_string_dummy import hsr_StringDummy
from hsr_flexbe_states.hsr_veiw_marker import hsr_ViewMarker
from hsr_flexbe_states.hsr_moveit_to_pose_goal_action_state import hsr_MoveitToPoseGoalAction
from hsr_flexbe_states.hsr_counting import hsr_Counting
from flexbe_states.wait_state import WaitState
from hsr_flexbe_states.hsr_xtion_point_client import hsr_XtionMoveClient
from hsr_flexbe_states.hsr_xtion_point_state import hsr_XtionPoint
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Oct 12 2018
@author: tabuchi isobe
'''
class prc_finalSM(Behavior):
	'''
	tem
	'''


	def __init__(self):
		super(prc_finalSM, self).__init__()
		self.name = 'prc_final'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:511 y:532, x:610 y:460
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:55 y:26
			OperatableStateMachine.add('tts_CollectLaserMicrophone',
										hsr_TtsInputParameter(text="OK. I will take something drink from dingin table.", language="en"),
										transitions={'completed': 'hsr_MoveBaseTakePlace'},
										autonomy={'completed': Autonomy.Off})

			# x:737 y:369
			OperatableStateMachine.add('hsr_HandClose',
										hsr_GrippingObject(grasp_force=0.7, mode=True),
										transitions={'continue': 'hsr_MoveBack', 'failed': 'hsr_MoveBack'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:254 y:27
			OperatableStateMachine.add('hsr_MoveBaseTakePlace',
										hsr_MoveBaseClient(pose_position_x=2.93275682249, pose_position_y=-1.96998477423, pose_orientation_z=0.999996209364, pose_orientation_w=0.00275340834021),
										transitions={'completed': 'hsr_MoveTakePlace'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'take_place'})

			# x:434 y:28
			OperatableStateMachine.add('hsr_MoveTakePlace',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_HandOpen1', 'failed': 'hsr_MoveTakePlace'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'take_place'})

			# x:1546 y:503
			OperatableStateMachine.add('hsr_HandOver',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=-3.14/4, arm_roll_joint=0.0, wrist_flex_joint=-3.14/4, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=-0.2),
										transitions={'continue': 'tts_Hereyouare'},
										autonomy={'continue': Autonomy.Off})

			# x:1370 y:505
			OperatableStateMachine.add('tts_Hereyouare',
										hsr_TtsInputParameter(text="Here you are.", language="en"),
										transitions={'completed': 'hsr_HandOpen'},
										autonomy={'completed': Autonomy.Off})

			# x:1188 y:506
			OperatableStateMachine.add('hsr_HandOpen',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_JointPoseGo2', 'failed': 'hsr_HandOpen'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:782 y:517
			OperatableStateMachine.add('hsr_MoveBaseStandPlace',
										hsr_MoveBaseClient(pose_position_x=2.19663095949, pose_position_y=-0.107342765678, pose_orientation_z=-0.135386180174, pose_orientation_w=0.990792905817),
										transitions={'completed': 'hsr_MoveStandPlace'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'standby_place'})

			# x:1017 y:516
			OperatableStateMachine.add('hsr_JointPoseGo2',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_MoveBaseStandPlace'},
										autonomy={'continue': Autonomy.Off})

			# x:1084 y:363
			OperatableStateMachine.add('hsr_JointPoseGo1',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_MoveBasePartyRoom'},
										autonomy={'continue': Autonomy.Off})

			# x:1347 y:208
			OperatableStateMachine.add('hsr_JointPoseNeutral',
										hsr_JointPose(arm_lift_joint=0.3, arm_flex_joint=-3.14/4, arm_roll_joint=0.0, wrist_flex_joint=-3.14/4, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_ObjectName'},
										autonomy={'continue': Autonomy.Off})

			# x:593 y:27
			OperatableStateMachine.add('hsr_HandOpen1',
										hsr_GrippingObject(grasp_force=0.7, mode=False),
										transitions={'continue': 'hsr_JointPose_xtion', 'failed': 'hsr_HandOpen1'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})

			# x:1261 y:19
			OperatableStateMachine.add('hsr_ImageCapture',
										hsr_ImageCapture(rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info"),
										transitions={'completed': 'hsr_BoundingBox2D', 'failed': 'hsr_ImageCapture'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info'})

			# x:1396 y:23
			OperatableStateMachine.add('hsr_BoundingBox2D',
										hsr_BoundingBox2D(output_topic="/bounding_box_2d_monitor"),
										transitions={'completed': 'hsr_GraspingPointDetect', 'failed': 'hsr_BoundingBox2D'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'detection': 'detection', 'grasping_point': 'grasping_point'})

			# x:1558 y:21
			OperatableStateMachine.add('hsr_GraspingPointDetect',
										hsr_GraspingPointDetect(output_topic="/bounding_box_2d_monitor"),
										transitions={'completed': 'hsr_Tf2', 'failed': 'hsr_GraspingPointDetect'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'detection': 'detection', 'grasping_point': 'grasping_point'})

			# x:1622 y:99
			OperatableStateMachine.add('hsr_Tf2',
										hsr_Tf2(before="head_rgbd_sensor_rgb_frame", after="map"),
										transitions={'continue': 'hsr_JointPoseNeutral', 'failed': 'hsr_Tf2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'before_pose': 'grasping_point', 'after_pose': 'after_pose'})

			# x:308 y:265
			OperatableStateMachine.add('hsr_PoseDecisionFinal',
										hsr_PoseDecisionConveni(offset=None),
										transitions={'continue': 'hsr_ViewMarker', 'plan_failed': 'tts_FailPlan', 'detect_failed': 'tts_FailDetection'},
										autonomy={'continue': Autonomy.Off, 'plan_failed': Autonomy.Off, 'detect_failed': Autonomy.Off},
										remapping={'target_poses': 'after_pose', 'target_object_names': 'object', 'failed_objects': 'object', 'selected_pose_approach': 'selected_pose_approach', 'selected_pose_grasp': 'selected_pose_grasp', 'selected_object_name': 'selected_object_name', 'selected_object_status': 'selected_object_status'})

			# x:1111 y:206
			OperatableStateMachine.add('hsr_ObjectName',
										hsr_StringDummy(text=["bottle"]),
										transitions={'continue': 'hsr_PoseDecisionFinal'},
										autonomy={'continue': Autonomy.Off},
										remapping={'name': 'object'})

			# x:22 y:147
			OperatableStateMachine.add('tts_FailPlan',
										hsr_TtsInputParameter(text="I failed to plan a trajectry, but I will try again.", language="en"),
										transitions={'completed': 'hsr_PlanTryCount'},
										autonomy={'completed': Autonomy.Off})

			# x:311 y:368
			OperatableStateMachine.add('hsr_ViewMarker',
										hsr_ViewMarker(),
										transitions={'continue': 'hsr_MoveitToPoseGoalAction', 'failed': 'hsr_ViewMarker'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'view_pose': 'selected_pose_grasp'})

			# x:492 y:368
			OperatableStateMachine.add('hsr_MoveitToPoseGoalAction',
										hsr_MoveitToPoseGoalAction(move_group='whole_body', action_topic='/move_group'),
										transitions={'reached': 'hsr_HandClose', 'planning_failed': 'hsr_PlanTryCount', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'pose_goal': 'selected_pose_grasp', 'move_status': 'move_status'})

			# x:917 y:366
			OperatableStateMachine.add('hsr_MoveBack',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_JointPoseGo1', 'failed': 'hsr_JointPoseGo1'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'take_place'})

			# x:1279 y:360
			OperatableStateMachine.add('hsr_MoveBasePartyRoom',
										hsr_MoveBaseClient(pose_position_x=4.78038886411, pose_position_y=1.95365056449, pose_orientation_z=-0.148683989838, pose_orientation_w=0.988884761317),
										transitions={'completed': 'hsr_MovePartyPlace'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'bring_place'})

			# x:348 y:165
			OperatableStateMachine.add('hsr_PlanTryCount',
										hsr_Counting(max_count=5),
										transitions={'repetition': 'hsr_XtionMoveClient', 'end': 'tts_fail_take_object'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off})

			# x:596 y:522
			OperatableStateMachine.add('hsr_MoveStandPlace',
										hsr_MoveBase(),
										transitions={'succeeded': 'finished', 'failed': 'hsr_MoveStandPlace'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'standby_place'})

			# x:641 y:165
			OperatableStateMachine.add('tts_fail_take_object',
										hsr_TtsInputParameter(text="I cannot detect the object, please handover the object.", language="en"),
										transitions={'completed': 'wait_5sec'},
										autonomy={'completed': Autonomy.Off})

			# x:835 y:277
			OperatableStateMachine.add('wait_5sec',
										WaitState(wait_time=5.0),
										transitions={'done': 'hsr_HandClose'},
										autonomy={'done': Autonomy.Off})

			# x:1525 y:360
			OperatableStateMachine.add('hsr_MovePartyPlace',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_HandOver', 'failed': 'hsr_MovePartyPlace'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'bring_place'})

			# x:966 y:22
			OperatableStateMachine.add('hsr_XtionMoveClient',
										hsr_XtionMoveClient(point_x=0.792080036183, point_y=-1.96998477423, point_z=0.776622437874),
										transitions={'completed': 'hsr_XtionPoint'},
										autonomy={'completed': Autonomy.Off},
										remapping={'point': 'point'})

			# x:1102 y:20
			OperatableStateMachine.add('hsr_XtionPoint',
										hsr_XtionPoint(ref_frame_id="map"),
										transitions={'finish': 'hsr_ImageCapture'},
										autonomy={'finish': Autonomy.Off},
										remapping={'xtion_point': 'point'})

			# x:769 y:23
			OperatableStateMachine.add('hsr_JointPose_xtion',
										hsr_JointPose(arm_lift_joint=0.3, arm_flex_joint=-2.619, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=0.0),
										transitions={'continue': 'hsr_XtionMoveClient'},
										autonomy={'continue': Autonomy.Off})

			# x:594 y:296
			OperatableStateMachine.add('tts_FailDetection',
										hsr_TtsInputParameter(text="I cannot see the object, but I will try again.", language="en"),
										transitions={'completed': 'wait_5sec'},
										autonomy={'completed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]

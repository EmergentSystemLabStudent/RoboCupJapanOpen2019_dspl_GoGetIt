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
from hsr_flexbe_states.hsr_label_arrangement_state import hsr_LabelArrangement
from hsr_flexbe_states.hsr_bounding_box_2d_state import hsr_BoundingBox2D
from hsr_flexbe_states.hsr_image_capture_state import hsr_ImageCapture
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 09 2019
@author: a
'''
class aSM(Behavior):
	'''
	a
	'''


	def __init__(self):
		super(aSM, self).__init__()
		self.name = 'a'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:38 y:102
			OperatableStateMachine.add('hsr_JointPoseMemo',
										hsr_JointPose(arm_lift_joint=0.4, arm_flex_joint=-3.14/3*2, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=0.0, head_tilt_joint=-0.35),
										transitions={'continue': 'hsr_ImageCaptureMemo'},
										autonomy={'continue': Autonomy.Off})

			# x:604 y:44
			OperatableStateMachine.add('hsr_LabelArrangement',
										hsr_LabelArrangement(path='/root/HSR/catkin_ws/src/em_spco_formation/training_data/default/place2object.txt'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'detection_result': 'detection_memo'})

			# x:386 y:44
			OperatableStateMachine.add('hsr_BoundingBox2DMemo',
										hsr_BoundingBox2D(output_topic="/bounding_box_2d_monitor"),
										transitions={'completed': 'hsr_LabelArrangement', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image_memo', 'depth_image': 'depth_image_memo', 'camera_info': 'camera_info_memo', 'detection': 'detection_memo'})

			# x:195 y:46
			OperatableStateMachine.add('hsr_ImageCaptureMemo',
										hsr_ImageCapture(rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info"),
										transitions={'completed': 'hsr_BoundingBox2DMemo', 'failed': 'failed'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image_memo', 'depth_image': 'depth_image_memo', 'camera_info': 'camera_info_memo'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

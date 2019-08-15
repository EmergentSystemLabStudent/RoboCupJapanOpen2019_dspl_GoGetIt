#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_string_dummy import hsr_StringDummy
from hsr_flexbe_states.hsr_image_capture_state import hsr_ImageCapture
from hsr_flexbe_states.hsr_bounding_box_2d_state import hsr_BoundingBox2D
from hsr_flexbe_states.hsr_grasping_point_detect_state import hsr_GraspingPointDetect
from hsr_flexbe_states.hsr_tf2_state import hsr_Tf2
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Aug 07 2019
@author: Hitoshi Nakamura
'''
class Test_object_graspSM(Behavior):
	'''
	Test_object_grasp
	'''


	def __init__(self):
		super(Test_object_graspSM, self).__init__()
		self.name = 'Test_object_grasp'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:478, x:130 y:478
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:478
		_sm_newest_product_0 = OperatableStateMachine(outcomes=['continue'], input_keys=['object'], output_keys=['selected_object_status'])

		with _sm_newest_product_0:
			# x:194 y:31
			OperatableStateMachine.add('hsr_ImageCapture',
										hsr_ImageCapture(rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info"),
										transitions={'completed': 'hsr_BoundingBox2D', 'failed': 'hsr_ImageCapture'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info'})

			# x:388 y:33
			OperatableStateMachine.add('hsr_BoundingBox2D',
										hsr_BoundingBox2D(output_topic='/bounding_box_2d_monitor'),
										transitions={'completed': 'hsr_GraspingPointDetect', 'failed': 'hsr_BoundingBox2D'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'detection': 'detection'})

			# x:548 y:31
			OperatableStateMachine.add('hsr_GraspingPointDetect',
										hsr_GraspingPointDetect(output_topic="/bounding_box_2d_monitor", save=True),
										transitions={'completed': 'hsr_Tf2', 'failed': 'hsr_GraspingPointDetect'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'detection': 'detection', 'grasping_point': 'grasping_point'})

			# x:740 y:31
			OperatableStateMachine.add('hsr_Tf2',
										hsr_Tf2(before='head_rgbd_sensor_rgb_frame', after='map'),
										transitions={'continue': 'continue', 'failed': 'hsr_Tf2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'before_pose': 'grasping_point', 'after_pose': 'after_pose'})



		with _state_machine:
			# x:23 y:114
			OperatableStateMachine.add('hsr_StringDummy',
										hsr_StringDummy(text="bottle"),
										transitions={'continue': 'Newest Product'},
										autonomy={'continue': Autonomy.Off},
										remapping={'name': 'object'})

			# x:216 y:109
			OperatableStateMachine.add('Newest Product',
										_sm_newest_product_0,
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Inherit},
										remapping={'object': 'object', 'selected_object_status': 'selected_object_status'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

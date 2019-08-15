#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_image_capture_state import hsr_ImageCapture
from hsr_flexbe_states.hsr_bounding_box_2d_state import hsr_BoundingBox2D
from hsr_flexbe_states.hsr_grasping_point_detect_state import hsr_GraspingPointDetect
from hsr_flexbe_states.hsr_tf2_state import hsr_Tf2
from hsr_flexbe_states.hsr_veiw_detection_state import hsr_ViewDetection
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Aug 11 2019
@author: Hitoshi Nakamura
'''
class Final_object_testSM(Behavior):
	'''
	Final_object_test
	'''


	def __init__(self):
		super(Final_object_testSM, self).__init__()
		self.name = 'Final_object_test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:967 y:198, x:232 y:377
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:478, x:130 y:478
		_sm_visualize_0 = OperatableStateMachine(outcomes=['failed', 'continue'])

		with _sm_visualize_0:
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



		with _state_machine:
			# x:314 y:82
			OperatableStateMachine.add('Visualize',
										_sm_visualize_0,
										transitions={'failed': 'failed', 'continue': 'finished'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

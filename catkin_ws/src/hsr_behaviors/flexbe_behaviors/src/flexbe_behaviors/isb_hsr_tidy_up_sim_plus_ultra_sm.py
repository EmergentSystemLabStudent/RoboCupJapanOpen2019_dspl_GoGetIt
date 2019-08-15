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
from hsr_flexbe_states.hsr_veiw_marker_array import hsr_ViewMarkerArray
from hsr_flexbe_states.hsr_set_object_state import hsr_SetObjectState
from hsr_flexbe_states.hsr_spacoty_instruction import hsr_SpaCoTyInstruction
from hsr_flexbe_states.hsr_get_object_states import hsr_GetObjectState
from hsr_flexbe_states.hsr_select_tidy_pose_randomly_from_train_dataset_of_spacoty_ultra import hsr_SelectTidyObjectandPoseRandomlyUltra
from hsr_flexbe_states.hsr_select_tidy_pose_from_train_dataset_of_spacoty_ultra import hsr_SelectTidyObjectandPosefromDatasetUltra
from hsr_flexbe_states.hsr_switcher import hsr_Switcher
from hsr_flexbe_states.hsr_spacoty_update_pose import hsr_SpaCoTyUpdateTidyPose
from hsr_flexbe_states.hsr_output_num import hsr_OutputNum
from hsr_flexbe_states.hsr_counting_add_reset import hsr_CountingAddReset
from hsr_flexbe_states.hsr_reset_object_states_randomly import hsr_ResetObjectStateRandomly
from hsr_flexbe_states.hsr_speech_recognition_state import hsr_SpeechRecognition
from hsr_flexbe_states.hsr_make_file import hsr_MakeFile
from hsr_flexbe_states.hsr_image_capture_state import hsr_ImageCapture
from hsr_flexbe_states.hsr_counting import hsr_Counting
from hsr_flexbe_states.hsr_grasping_point_detect_state_isb_plus import hsr_GraspingPointDetectIsbPlus
from hsr_flexbe_states.hsr_spacoty_planning_plus import hsr_SpaCoTyPlanningPlus
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Oct 10 2018
@author: shota isobe
'''
class isb_hsr_tidy_up_sim_plus_ultraSM(Behavior):
	'''
	isb_hsr_tidy_up_sim_plus_ultra
	'''


	def __init__(self):
		super(isb_hsr_tidy_up_sim_plus_ultraSM, self).__init__()
		self.name = 'isb_hsr_tidy_up_sim_plus_ultra'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1258 y:287, x:1255 y:399
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:54 y:72
			OperatableStateMachine.add('hsr_JointPose_FindObject',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=0.0, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=-1.3, head_tilt_joint=-0.8),
										transitions={'continue': 'hsr_SpeechRecognition'},
										autonomy={'continue': Autonomy.Off})

			# x:1046 y:79
			OperatableStateMachine.add('hsr_ViewMarkerArray_1',
										hsr_ViewMarkerArray(),
										transitions={'continue': 'hsr_ImageCapture'},
										autonomy={'continue': Autonomy.Off},
										remapping={'object_points': 'greeeedy'})

			# x:272 y:163
			OperatableStateMachine.add('hsr_SetObjectState_1',
										hsr_SetObjectState(),
										transitions={'completed': 'hsr_CountingAddReset_1', 'failed': 'hsr_SetObjectState_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_name': 'tidy_order', 'object_pose': 'tidy_pose'})

			# x:1046 y:332
			OperatableStateMachine.add('hsr_SpaCoTyInstruction',
										hsr_SpaCoTyInstruction(save_folder="sim_st1"),
										transitions={'datadata': 'finished', 'training': 'finished', 'tidy_baseline_nearly': 'hsr_MakeFile_Near', 'tidy_baseline_randomly': 'hsr_MakeFile_Random', 'tidy_greedy': 'hsr_MakeFile_Greedy', 'failed': 'failed', 'end': 'finished'},
										autonomy={'datadata': Autonomy.Off, 'training': Autonomy.Off, 'tidy_baseline_nearly': Autonomy.Off, 'tidy_baseline_randomly': Autonomy.Off, 'tidy_greedy': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'sentence': 'sentence', 'move_name': 'move_name', 'save_folder': 'save_folder'})

			# x:914 y:483
			OperatableStateMachine.add('hsr_GetObjectState_2',
										hsr_GetObjectState(yolo_yaml="yolov3-isb-tidy.json"),
										transitions={'completed': 'hsr_ViewMarkerArray_2', 'failed': 'hsr_GetObjectState_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'load_file': 'load_file', 'object_states': 'neaaaar', 'object_states_init': 'init_neaaar'})

			# x:1055 y:644
			OperatableStateMachine.add('hsr_GetObjectState_3',
										hsr_GetObjectState(yolo_yaml="yolov3-isb-tidy.json"),
										transitions={'completed': 'hsr_ViewMarkerArray_3', 'failed': 'hsr_GetObjectState_3'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'load_file': 'load_file', 'object_states': 'randommmm', 'object_states_init': 'init_randommm'})

			# x:723 y:482
			OperatableStateMachine.add('hsr_ViewMarkerArray_2',
										hsr_ViewMarkerArray(),
										transitions={'continue': 'hsr_SelectTidyObjectandPosefromDatasetUltra'},
										autonomy={'continue': Autonomy.Off},
										remapping={'object_points': 'neaaaar'})

			# x:805 y:644
			OperatableStateMachine.add('hsr_ViewMarkerArray_3',
										hsr_ViewMarkerArray(),
										transitions={'continue': 'hsr_SelectTidyObjectandPoseRandomlyUltra'},
										autonomy={'continue': Autonomy.Off},
										remapping={'object_points': 'randommmm'})

			# x:254 y:478
			OperatableStateMachine.add('hsr_SetObjectState_2',
										hsr_SetObjectState(),
										transitions={'completed': 'hsr_CountingAddReset_2', 'failed': 'hsr_SetObjectState_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_name': 'tidy_name_db', 'object_pose': 'tidy_pose_db'})

			# x:316 y:642
			OperatableStateMachine.add('hsr_SetObjectState_3',
										hsr_SetObjectState(),
										transitions={'completed': 'hsr_CountingAddReset_3', 'failed': 'hsr_SetObjectState_3'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_name': 'tidy_name_random', 'object_pose': 'tidy_pose_random'})

			# x:1051 y:157
			OperatableStateMachine.add('hsr_GetObjectState_1',
										hsr_GetObjectState(yolo_yaml="yolov3-isb-tidy.json"),
										transitions={'completed': 'hsr_ViewMarkerArray_1', 'failed': 'hsr_GetObjectState_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'load_file': 'load_file', 'object_states': 'greeeedy', 'object_states_init': 'init_greeedy'})

			# x:510 y:643
			OperatableStateMachine.add('hsr_SelectTidyObjectandPoseRandomlyUltra',
										hsr_SelectTidyObjectandPoseRandomlyUltra(),
										transitions={'completed': 'hsr_SetObjectState_3'},
										autonomy={'completed': Autonomy.Off},
										remapping={'load_file': 'load_file', 'detect_objects': 'randommmm', 'tidy_name_random': 'tidy_name_random', 'tidy_pose_random': 'tidy_pose_random'})

			# x:424 y:481
			OperatableStateMachine.add('hsr_SelectTidyObjectandPosefromDatasetUltra',
										hsr_SelectTidyObjectandPosefromDatasetUltra(),
										transitions={'completed': 'hsr_SetObjectState_2'},
										autonomy={'completed': Autonomy.Off},
										remapping={'load_file': 'load_file', 'detect_objects': 'neaaaar', 'tidy_name_db': 'tidy_name_db', 'tidy_pose_db': 'tidy_pose_db'})

			# x:523 y:163
			OperatableStateMachine.add('hsr_Switcher',
										hsr_Switcher(),
										transitions={'true_load': 'hsr_SetObjectState_1', 'false_load': 'hsr_SetObjectState_1'},
										autonomy={'true_load': Autonomy.Off, 'false_load': Autonomy.Off},
										remapping={'switch': 'name2place'})

			# x:258 y:83
			OperatableStateMachine.add('hsr_SpaCoTyUpdateTidyPose',
										hsr_SpaCoTyUpdateTidyPose(),
										transitions={'completed': 'hsr_SetObjectState_1', 'failed': 'hsr_SpaCoTyUpdateTidyPose'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_name': 'tidy_order', 'tidy_pose_bef': 'tidy_pose', 'tidy_pose_aft': 'tidy_pose'})

			# x:579 y:329
			OperatableStateMachine.add('hsr_OutputNum',
										hsr_OutputNum(num=0),
										transitions={'end': 'hsr_SpaCoTyInstruction'},
										autonomy={'end': Autonomy.Off},
										remapping={'count_num': 'init_num'})

			# x:707 y:247
			OperatableStateMachine.add('hsr_CountingAddReset_1',
										hsr_CountingAddReset(max_count=10),
										transitions={'repetition': 'hsr_GetObjectState_1', 'end': 'hsr_ResetObjectStateRandomly_1'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'start_num': 'init_num', 'count_num': 'count_num'})

			# x:583 y:401
			OperatableStateMachine.add('hsr_CountingAddReset_2',
										hsr_CountingAddReset(max_count=10),
										transitions={'repetition': 'hsr_GetObjectState_2', 'end': 'hsr_ResetObjectStateRandomly_2'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'start_num': 'init_num', 'count_num': 'count_num'})

			# x:308 y:553
			OperatableStateMachine.add('hsr_CountingAddReset_3',
										hsr_CountingAddReset(max_count=10),
										transitions={'repetition': 'hsr_GetObjectState_3', 'end': 'hsr_ResetObjectStateRandomly_3'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'start_num': 'init_num', 'count_num': 'count_num'})

			# x:44 y:642
			OperatableStateMachine.add('hsr_ResetObjectStateRandomly_3',
										hsr_ResetObjectStateRandomly(),
										transitions={'completed': 'hsr_SpeechRecognition', 'failed': 'hsr_ResetObjectStateRandomly_3'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_states': 'init_randommm'})

			# x:211 y:399
			OperatableStateMachine.add('hsr_ResetObjectStateRandomly_2',
										hsr_ResetObjectStateRandomly(),
										transitions={'completed': 'hsr_SpeechRecognition', 'failed': 'hsr_ResetObjectStateRandomly_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_states': 'init_neaaar'})

			# x:256 y:244
			OperatableStateMachine.add('hsr_ResetObjectStateRandomly_1',
										hsr_ResetObjectStateRandomly(),
										transitions={'completed': 'hsr_SpeechRecognition', 'failed': 'hsr_ResetObjectStateRandomly_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_states': 'init_greeedy'})

			# x:58 y:326
			OperatableStateMachine.add('hsr_SpeechRecognition',
										hsr_SpeechRecognition(topic="julius/speech2text/en"),
										transitions={'recognition': 'hsr_OutputNum'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'sentence'})

			# x:921 y:403
			OperatableStateMachine.add('hsr_MakeFile_Near',
										hsr_MakeFile(save_file="sim_near_ultra"),
										transitions={'completed': 'hsr_GetObjectState_2'},
										autonomy={'completed': Autonomy.Off},
										remapping={'load_file': 'load_file'})

			# x:1051 y:553
			OperatableStateMachine.add('hsr_MakeFile_Random',
										hsr_MakeFile(save_file="sim_random_ultra"),
										transitions={'completed': 'hsr_GetObjectState_3'},
										autonomy={'completed': Autonomy.Off},
										remapping={'load_file': 'load_file'})

			# x:1054 y:235
			OperatableStateMachine.add('hsr_MakeFile_Greedy',
										hsr_MakeFile(save_file="plus_sim_greedy_ultra"),
										transitions={'completed': 'hsr_GetObjectState_1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'load_file': 'load_file'})

			# x:900 y:79
			OperatableStateMachine.add('hsr_ImageCapture',
										hsr_ImageCapture(rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info"),
										transitions={'completed': 'hsr_GraspingPointDetectIsbPlus', 'failed': 'hsr_CaptureTime'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info'})

			# x:902 y:15
			OperatableStateMachine.add('hsr_CaptureTime',
										hsr_Counting(max_count=2),
										transitions={'repetition': 'hsr_CaptureTime', 'end': 'hsr_ImageCapture'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'count_num': 'capture_num'})

			# x:689 y:13
			OperatableStateMachine.add('hsr_DetectErrorIgnoreOnce',
										hsr_Counting(max_count=2),
										transitions={'repetition': 'hsr_DetectErrorIgnoreOnce', 'end': 'hsr_GraspingPointDetectIsbPlus'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'count_num': '_num'})

			# x:679 y:83
			OperatableStateMachine.add('hsr_GraspingPointDetectIsbPlus',
										hsr_GraspingPointDetectIsbPlus(output_topic="/bounding_box_2d_monitor", resnet_model="resnet50", save=True),
										transitions={'completed': 'hsr_SpaCoTyPlanningPlus', 'failed': 'hsr_GraspingPointDetectIsbPlus', 'yolo_failed': 'hsr_DetectErrorIgnoreOnce'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off, 'yolo_failed': Autonomy.Off},
										remapping={'rgb_image': 'rgb_image', 'depth_image': 'depth_image', 'camera_info': 'camera_info', 'grasping_point': 'grasping_point', 'grasping_point_feature': 'grasping_point_feature'})

			# x:496 y:83
			OperatableStateMachine.add('hsr_SpaCoTyPlanningPlus',
										hsr_SpaCoTyPlanningPlus(),
										transitions={'completed': 'hsr_Switcher'},
										autonomy={'completed': Autonomy.Off},
										remapping={'load_file': 'load_file', 'detect_objects': 'greeeedy', 'detect_objects_feature': 'grasping_point_feature', 'tidy_pose': 'tidy_pose', 'tidy_order': 'tidy_order', 'name2place': 'name2place'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]

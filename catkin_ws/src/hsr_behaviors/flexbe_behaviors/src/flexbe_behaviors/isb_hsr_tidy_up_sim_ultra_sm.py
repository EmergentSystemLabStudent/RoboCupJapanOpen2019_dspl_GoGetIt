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
from hsr_flexbe_states.hsr_spacoty_planning import hsr_SpaCoTyPlanning
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Oct 10 2018
@author: shota isobe
'''
class isb_hsr_tidy_up_sim_ultraSM(Behavior):
	'''
	isb_hsr_tidy_up_sim_ultra
	'''


	def __init__(self):
		super(isb_hsr_tidy_up_sim_ultraSM, self).__init__()
		self.name = 'isb_hsr_tidy_up_sim_ultra'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1242 y:251, x:1219 y:335
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:70 y:29
			OperatableStateMachine.add('hsr_JointPose_FindObject',
										hsr_JointPose(arm_lift_joint=0.0, arm_flex_joint=0.0, arm_roll_joint=-3.14/2, wrist_flex_joint=-3.14/2, wrist_roll_joint=0.0, head_pan_joint=-1.3, head_tilt_joint=-0.8),
										transitions={'continue': 'hsr_SpeechRecognition'},
										autonomy={'continue': Autonomy.Off})

			# x:949 y:83
			OperatableStateMachine.add('hsr_ViewMarkerArray_1',
										hsr_ViewMarkerArray(),
										transitions={'continue': 'hsr_SpaCoTyPlanning'},
										autonomy={'continue': Autonomy.Off},
										remapping={'object_points': 'greeeedy'})

			# x:285 y:75
			OperatableStateMachine.add('hsr_SetObjectState_1',
										hsr_SetObjectState(),
										transitions={'completed': 'hsr_CountingAddReset_1', 'failed': 'hsr_SetObjectState_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_name': 'tidy_order', 'object_pose': 'tidy_pose'})

			# x:1047 y:240
			OperatableStateMachine.add('hsr_SpaCoTyInstruction',
										hsr_SpaCoTyInstruction(save_folder="sim_st1"),
										transitions={'datadata': 'finished', 'training': 'finished', 'tidy_baseline_nearly': 'hsr_MakeFile_Near', 'tidy_baseline_randomly': 'hsr_MakeFile_Random', 'tidy_greedy': 'hsr_MakeFile_Greedy', 'failed': 'failed', 'end': 'finished'},
										autonomy={'datadata': Autonomy.Off, 'training': Autonomy.Off, 'tidy_baseline_nearly': Autonomy.Off, 'tidy_baseline_randomly': Autonomy.Off, 'tidy_greedy': Autonomy.Off, 'failed': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'sentence': 'sentence', 'move_name': 'move_name', 'save_folder': 'save_folder'})

			# x:914 y:409
			OperatableStateMachine.add('hsr_GetObjectState_2',
										hsr_GetObjectState(yolo_yaml="yolov3-isb-tidy.json"),
										transitions={'completed': 'hsr_ViewMarkerArray_2', 'failed': 'hsr_GetObjectState_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'load_file': 'load_file', 'object_states': 'neaaaar', 'object_states_init': 'init_neaaar'})

			# x:999 y:616
			OperatableStateMachine.add('hsr_GetObjectState_3',
										hsr_GetObjectState(yolo_yaml="yolov3-isb-tidy.json"),
										transitions={'completed': 'hsr_ViewMarkerArray_3', 'failed': 'hsr_GetObjectState_3'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'load_file': 'load_file', 'object_states': 'randommmm', 'object_states_init': 'init_randommm'})

			# x:723 y:407
			OperatableStateMachine.add('hsr_ViewMarkerArray_2',
										hsr_ViewMarkerArray(),
										transitions={'continue': 'hsr_SelectTidyObjectandPosefromDatasetUltra'},
										autonomy={'continue': Autonomy.Off},
										remapping={'object_points': 'neaaaar'})

			# x:739 y:615
			OperatableStateMachine.add('hsr_ViewMarkerArray_3',
										hsr_ViewMarkerArray(),
										transitions={'continue': 'hsr_SelectTidyObjectandPoseRandomlyUltra'},
										autonomy={'continue': Autonomy.Off},
										remapping={'object_points': 'randommmm'})

			# x:251 y:400
			OperatableStateMachine.add('hsr_SetObjectState_2',
										hsr_SetObjectState(),
										transitions={'completed': 'hsr_CountingAddReset_2', 'failed': 'hsr_SetObjectState_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_name': 'tidy_name_db', 'object_pose': 'tidy_pose_db'})

			# x:265 y:612
			OperatableStateMachine.add('hsr_SetObjectState_3',
										hsr_SetObjectState(),
										transitions={'completed': 'hsr_CountingAddReset_3', 'failed': 'hsr_SetObjectState_3'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_name': 'tidy_name_random', 'object_pose': 'tidy_pose_random'})

			# x:953 y:152
			OperatableStateMachine.add('hsr_GetObjectState_1',
										hsr_GetObjectState(yolo_yaml="yolov3-isb-tidy.json"),
										transitions={'completed': 'hsr_ViewMarkerArray_1', 'failed': 'hsr_GetObjectState_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'load_file': 'load_file', 'object_states': 'greeeedy', 'object_states_init': 'init_greeedy'})

			# x:441 y:614
			OperatableStateMachine.add('hsr_SelectTidyObjectandPoseRandomlyUltra',
										hsr_SelectTidyObjectandPoseRandomlyUltra(),
										transitions={'completed': 'hsr_SetObjectState_3'},
										autonomy={'completed': Autonomy.Off},
										remapping={'load_file': 'load_file', 'detect_objects': 'randommmm', 'tidy_name_random': 'tidy_name_random', 'tidy_pose_random': 'tidy_pose_random'})

			# x:421 y:405
			OperatableStateMachine.add('hsr_SelectTidyObjectandPosefromDatasetUltra',
										hsr_SelectTidyObjectandPosefromDatasetUltra(),
										transitions={'completed': 'hsr_SetObjectState_2'},
										autonomy={'completed': Autonomy.Off},
										remapping={'load_file': 'load_file', 'detect_objects': 'neaaaar', 'tidy_name_db': 'tidy_name_db', 'tidy_pose_db': 'tidy_pose_db'})

			# x:659 y:82
			OperatableStateMachine.add('hsr_Switcher',
										hsr_Switcher(),
										transitions={'true_load': 'hsr_SpaCoTyUpdateTidyPose', 'false_load': 'hsr_SetObjectState_1'},
										autonomy={'true_load': Autonomy.Off, 'false_load': Autonomy.Off},
										remapping={'switch': 'name2place'})

			# x:464 y:80
			OperatableStateMachine.add('hsr_SpaCoTyUpdateTidyPose',
										hsr_SpaCoTyUpdateTidyPose(),
										transitions={'completed': 'hsr_SetObjectState_1', 'failed': 'hsr_SpaCoTyUpdateTidyPose'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_name': 'tidy_order', 'tidy_pose_bef': 'tidy_pose', 'tidy_pose_aft': 'tidy_pose'})

			# x:581 y:238
			OperatableStateMachine.add('hsr_OutputNum',
										hsr_OutputNum(num=0),
										transitions={'end': 'hsr_SpaCoTyInstruction'},
										autonomy={'end': Autonomy.Off},
										remapping={'count_num': 'init_num'})

			# x:584 y:160
			OperatableStateMachine.add('hsr_CountingAddReset_1',
										hsr_CountingAddReset(max_count=10),
										transitions={'repetition': 'hsr_GetObjectState_1', 'end': 'hsr_ResetObjectStateRandomly_1'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'start_num': 'init_num', 'count_num': 'count_num'})

			# x:581 y:311
			OperatableStateMachine.add('hsr_CountingAddReset_2',
										hsr_CountingAddReset(max_count=10),
										transitions={'repetition': 'hsr_GetObjectState_2', 'end': 'hsr_ResetObjectStateRandomly_2'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'start_num': 'init_num', 'count_num': 'count_num'})

			# x:465 y:506
			OperatableStateMachine.add('hsr_CountingAddReset_3',
										hsr_CountingAddReset(max_count=10),
										transitions={'repetition': 'hsr_GetObjectState_3', 'end': 'hsr_ResetObjectStateRandomly_3'},
										autonomy={'repetition': Autonomy.Off, 'end': Autonomy.Off},
										remapping={'start_num': 'init_num', 'count_num': 'count_num'})

			# x:26 y:609
			OperatableStateMachine.add('hsr_ResetObjectStateRandomly_3',
										hsr_ResetObjectStateRandomly(),
										transitions={'completed': 'hsr_SpeechRecognition', 'failed': 'hsr_ResetObjectStateRandomly_3'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_states': 'init_randommm'})

			# x:221 y:314
			OperatableStateMachine.add('hsr_ResetObjectStateRandomly_2',
										hsr_ResetObjectStateRandomly(),
										transitions={'completed': 'hsr_SpeechRecognition', 'failed': 'hsr_ResetObjectStateRandomly_2'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_states': 'init_neaaar'})

			# x:254 y:158
			OperatableStateMachine.add('hsr_ResetObjectStateRandomly_1',
										hsr_ResetObjectStateRandomly(),
										transitions={'completed': 'hsr_SpeechRecognition', 'failed': 'hsr_ResetObjectStateRandomly_1'},
										autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object_states': 'init_greeedy'})

			# x:71 y:239
			OperatableStateMachine.add('hsr_SpeechRecognition',
										hsr_SpeechRecognition(topic="julius/speech2text/en"),
										transitions={'recognition': 'hsr_OutputNum'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'sentence'})

			# x:934 y:317
			OperatableStateMachine.add('hsr_MakeFile_Near',
										hsr_MakeFile(save_file="sim_near_ultra"),
										transitions={'completed': 'hsr_GetObjectState_2'},
										autonomy={'completed': Autonomy.Off},
										remapping={'load_file': 'load_file'})

			# x:995 y:512
			OperatableStateMachine.add('hsr_MakeFile_Random',
										hsr_MakeFile(save_file="sim_random_ultra"),
										transitions={'completed': 'hsr_GetObjectState_3'},
										autonomy={'completed': Autonomy.Off},
										remapping={'load_file': 'load_file'})

			# x:1096 y:151
			OperatableStateMachine.add('hsr_MakeFile_Greedy',
										hsr_MakeFile(save_file="sim_greedy_ultra"),
										transitions={'completed': 'hsr_GetObjectState_1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'load_file': 'load_file'})

			# x:794 y:82
			OperatableStateMachine.add('hsr_SpaCoTyPlanning',
										hsr_SpaCoTyPlanning(),
										transitions={'completed': 'hsr_Switcher'},
										autonomy={'completed': Autonomy.Off},
										remapping={'load_file': 'load_file', 'detect_objects': 'greeeedy', 'tidy_pose': 'tidy_pose', 'tidy_order': 'tidy_order', 'name2place': 'name2place'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]

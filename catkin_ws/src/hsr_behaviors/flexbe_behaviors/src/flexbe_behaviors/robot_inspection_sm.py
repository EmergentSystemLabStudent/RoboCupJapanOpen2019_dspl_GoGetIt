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
from hsr_flexbe_states.hsr_tts_input_parameter_state import hsr_TtsInputParameter
from hsr_flexbe_states.hsr_door_detect_state import hsr_DoorDetect
from hsr_flexbe_states.hsr_goal_pose_dummy_state import hsr_GoalPoseDummy
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBase
from hsr_flexbe_states.hsr_move_ahead_collision import hsr_Move_Ahead_Collision
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 24 2019
@author: Kazuma Furukawa
'''
class Robot_InspectionSM(Behavior):
	'''
	Robot_Inspection
	'''


	def __init__(self):
		super(Robot_InspectionSM, self).__init__()
		self.name = 'Robot_Inspection'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:849 y:279, x:1247 y:39
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:368 y:58
		_sm_gotoexit_0 = OperatableStateMachine(outcomes=['completed'], input_keys=['exit_pose'])

		with _sm_gotoexit_0:
			# x:41 y:40
			OperatableStateMachine.add('tts_EneterField',
										hsr_TtsInputParameter(text="I will go to the exit.", language="en", delay=1.0),
										transitions={'completed': 'hsr_MoveExit'},
										autonomy={'completed': Autonomy.Off})

			# x:222 y:38
			OperatableStateMachine.add('hsr_MoveExit',
										hsr_MoveBase(),
										transitions={'succeeded': 'completed', 'failed': 'hsr_MoveExit'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'exit_pose'})


		# x:30 y:478
		_sm_dooropen_1 = OperatableStateMachine(outcomes=['completed'])

		with _sm_dooropen_1:
			# x:30 y:45
			OperatableStateMachine.add('hsr_DoorDetect',
										hsr_DoorDetect(),
										transitions={'open': 'tts_DoorOpened', 'close': 'hsr_DoorDetect'},
										autonomy={'open': Autonomy.Off, 'close': Autonomy.Off})

			# x:206 y:40
			OperatableStateMachine.add('tts_DoorOpened',
										hsr_TtsInputParameter(text="Door opened.", language="en", delay=3),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off})



		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('hsr_MoveNeutral',
										hsr_MoveNeutral(),
										transitions={'continue': 'tts_Ready'},
										autonomy={'continue': Autonomy.Off})

			# x:178 y:40
			OperatableStateMachine.add('tts_Ready',
										hsr_TtsInputParameter(text="I'm ready!", language="en", delay=0),
										transitions={'completed': 'DoorOpen'},
										autonomy={'completed': Autonomy.Off})

			# x:363 y:38
			OperatableStateMachine.add('DoorOpen',
										_sm_dooropen_1,
										transitions={'completed': 'hsr_Move_Ahead_Collision'},
										autonomy={'completed': Autonomy.Inherit})

			# x:846 y:36
			OperatableStateMachine.add('hsr_ExitPose',
										hsr_GoalPoseDummy(px=3.93524549232, py=5.01833559259, pz=0.0, qx=0.0, qy=0.0, qz=0.687634546984, qw=0.726056974207),
										transitions={'continue': 'GoToExit'},
										autonomy={'continue': Autonomy.Off},
										remapping={'goal_pose': 'exit_pose'})

			# x:1038 y:29
			OperatableStateMachine.add('GoToExit',
										_sm_gotoexit_0,
										transitions={'completed': 'failed'},
										autonomy={'completed': Autonomy.Inherit},
										remapping={'exit_pose': 'exit_pose'})

			# x:651 y:39
			OperatableStateMachine.add('tts_EneterField',
										hsr_TtsInputParameter(text="I entered the field.", language="en", delay=1.0),
										transitions={'completed': 'hsr_ExitPose'},
										autonomy={'completed': Autonomy.Off})

			# x:496 y:132
			OperatableStateMachine.add('hsr_Move_Ahead_Collision',
										hsr_Move_Ahead_Collision(speed=0.3, ros_rate=30, distance=1.1),
										transitions={'continue': 'tts_EneterField'},
										autonomy={'continue': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

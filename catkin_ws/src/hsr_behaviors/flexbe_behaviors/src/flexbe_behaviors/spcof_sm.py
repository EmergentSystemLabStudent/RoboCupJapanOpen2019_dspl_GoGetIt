#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_speech_recognition_state import hsr_SpeechRecognition
from hsr_flexbe_states.hsr_spcof_training_state import hsr_SpcofTraining
from hsr_flexbe_states.hsr_spcof_instruction import hsr_SpcofInstruction
from hsr_flexbe_states.hsr_spcof_move_state import hsr_SpcofMove
from hsr_flexbe_states.hsr_tts_input_key_state import hsr_TtsInputKey
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBase
from hsr_flexbe_states.hsr_tts_input_parameter_state import hsr_TtsInputParameter
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Oct 19 2018
@author: tabuchi
'''
class spcofSM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(spcofSM, self).__init__()
		self.name = 'spcof'

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
			# x:412 y:45
			OperatableStateMachine.add('hsr_SpeechRecognition',
										hsr_SpeechRecognition(topic="/julius/speech2text/en"),
										transitions={'recognition': 'hsr_SpcofInstruction'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:148 y:287
			OperatableStateMachine.add('hsr_SpcofTraining',
										hsr_SpcofTraining(),
										transitions={'completed': 'hsr_training'},
										autonomy={'completed': Autonomy.Off},
										remapping={'request': 'output', 'spcof_text': 'spcof_text'})

			# x:414 y:291
			OperatableStateMachine.add('hsr_SpcofInstruction',
										hsr_SpcofInstruction(),
										transitions={'training': 'hsr_SpcofTraining', 'move': 'hsr_SpcofMove'},
										autonomy={'training': Autonomy.Off, 'move': Autonomy.Off},
										remapping={'input': 'recognition', 'output': 'output'})

			# x:710 y:289
			OperatableStateMachine.add('hsr_SpcofMove',
										hsr_SpcofMove(),
										transitions={'move': 'tts_move', 'failed': 'tts_cannot_move_2'},
										autonomy={'move': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_name': 'output', 'move_text': 'move_text', 'move_pose': 'move_pose', 'arrive_text': 'arrive_text'})

			# x:174 y:167
			OperatableStateMachine.add('hsr_training',
										hsr_TtsInputKey(),
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'spcof_text'})

			# x:821 y:198
			OperatableStateMachine.add('tts_move',
										hsr_TtsInputKey(),
										transitions={'completed': 'hsr_MoveBase'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'move_text'})

			# x:826 y:85
			OperatableStateMachine.add('hsr_MoveBase',
										hsr_MoveBase(),
										transitions={'succeeded': 'tts_arrive', 'failed': 'tts_cannot_move_1'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:630 y:17
			OperatableStateMachine.add('tts_arrive',
										hsr_TtsInputKey(),
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'arrive_text'})

			# x:633 y:192
			OperatableStateMachine.add('tts_cannot_move_2',
										hsr_TtsInputKey(),
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'move_text'})

			# x:626 y:101
			OperatableStateMachine.add('tts_cannot_move_1',
										hsr_TtsInputParameter(text="I cannot move there.", language="en"),
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

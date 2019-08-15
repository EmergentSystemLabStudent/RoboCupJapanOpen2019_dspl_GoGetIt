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
from hsr_flexbe_states.hsr_spcof_move_state import hsr_SpcofMove
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBase
from hsr_flexbe_states.hsr_tts_input_key_state import hsr_TtsInputKey
from hsr_flexbe_states.hsr_spcof_training_state import hsr_SpcofTraining
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 13 2019
@author: nakamura
'''
class spcof_demoSM(Behavior):
	'''
	spcof_demo
	'''


	def __init__(self):
		super(spcof_demoSM, self).__init__()
		self.name = 'spcof_demo'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:260 y:496, x:32 y:439
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:65 y:30
			OperatableStateMachine.add('hsr_SpeechRecognition',
										hsr_SpeechRecognition(topic="julius"),
										transitions={'recognition': 'hsr_SpcofTraining'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:472 y:346
			OperatableStateMachine.add('hsr_SpcofMove',
										hsr_SpcofMove(),
										transitions={'move': 'tts2', 'failed': 'hsr_SpcofMove'},
										autonomy={'move': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_name': 'recognition', 'move_text': 'move_text', 'move_pose': 'move_pose', 'arrive_text': 'arrive_text'})

			# x:630 y:463
			OperatableStateMachine.add('hsr_MoveBase',
										hsr_MoveBase(),
										transitions={'succeeded': 'tts3', 'failed': 'hsr_MoveBase'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:498 y:122
			OperatableStateMachine.add('tts1',
										hsr_TtsInputKey(),
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'spcof_text'})

			# x:634 y:376
			OperatableStateMachine.add('tts2',
										hsr_TtsInputKey(),
										transitions={'completed': 'hsr_MoveBase'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'move_text'})

			# x:631 y:560
			OperatableStateMachine.add('tts3',
										hsr_TtsInputKey(),
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'arrive_text'})

			# x:483 y:27
			OperatableStateMachine.add('hsr_SpcofTraining',
										hsr_SpcofTraining(),
										transitions={'completed': 'tts1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'request': 'recognition', 'spcof_text': 'spcof_text'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

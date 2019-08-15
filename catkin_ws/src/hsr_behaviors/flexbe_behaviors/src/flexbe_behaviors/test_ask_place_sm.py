#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_ask_place_state import hsr_AskPlace
from hsr_flexbe_states.hsr_tts_input_key_state import hsr_TtsInputKey
from hsr_flexbe_states.hsr_speech_recognition_state import hsr_SpeechRecognition
from hsr_flexbe_states.hsr_yes_no_ask_state import hsr_YesNoAsk
from hsr_flexbe_states.hsr_ask_object_state import hsr_AskObject
from hsr_flexbe_states.hsr_read_task_state import hsr_ReadTask
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 10 2019
@author: HitoshiNakamura
'''
class Test_ask_placeSM(Behavior):
	'''
	Test_ask_place
	'''


	def __init__(self):
		super(Test_ask_placeSM, self).__init__()
		self.name = 'Test_ask_place'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:745 y:443, x:130 y:478
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:67 y:42
			OperatableStateMachine.add('hsr_AskPlace',
										hsr_AskPlace(),
										transitions={'continue': 'tts_ask_place'},
										autonomy={'continue': Autonomy.Off},
										remapping={'place_name': 'place_name', 'question': 'question', 'place_number': 'place_number'})

			# x:284 y:43
			OperatableStateMachine.add('tts_ask_place',
										hsr_TtsInputKey(),
										transitions={'completed': 'hsr_SpeechRecognition3'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'question'})

			# x:447 y:38
			OperatableStateMachine.add('hsr_SpeechRecognition3',
										hsr_SpeechRecognition(topic="/julius/speech2text/en"),
										transitions={'recognition': 'hsr_YesNoAsk'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:283 y:158
			OperatableStateMachine.add('hsr_YesNoAsk',
										hsr_YesNoAsk(),
										transitions={'yes': 'hsr_ReadTask', 'no': 'hsr_AskPlace'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:687 y:217
			OperatableStateMachine.add('hsr_AskObject',
										hsr_AskObject(),
										transitions={'continue': 'tts_AskObject'},
										autonomy={'continue': Autonomy.Off},
										remapping={'object_list': 'target_list', 'object_name': 'object_name', 'question': 'question'})

			# x:507 y:218
			OperatableStateMachine.add('hsr_ReadTask',
										hsr_ReadTask(),
										transitions={'continue': 'hsr_AskObject', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'place_num': 'place_number', 'target_list': 'target_list'})

			# x:836 y:212
			OperatableStateMachine.add('tts_AskObject',
										hsr_TtsInputKey(),
										transitions={'completed': 'hsr_SpeechRecognition4'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'question'})

			# x:994 y:209
			OperatableStateMachine.add('hsr_SpeechRecognition4',
										hsr_SpeechRecognition(topic="/julius/speech2text/en"),
										transitions={'recognition': 'hsr_YesNoAsk_2'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:700 y:327
			OperatableStateMachine.add('hsr_YesNoAsk_2',
										hsr_YesNoAsk(),
										transitions={'yes': 'finished', 'no': 'hsr_AskObject'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off},
										remapping={'recognition': 'recognition'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

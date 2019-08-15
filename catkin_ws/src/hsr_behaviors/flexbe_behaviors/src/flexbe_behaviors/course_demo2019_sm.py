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
from hsr_flexbe_states.hsr_spcof_training_state import hsr_SpcofTraining
from hsr_flexbe_states.hsr_tts_input_key_state import hsr_TtsInputKey
from hsr_flexbe_states.hsr_spcof_move_state import hsr_SpcofMove
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBase
from hsr_flexbe_states.hsr_tts_input_parameter_state import hsr_TtsInputParameter
from hsr_flexbe_states.hsr_try_limit_state import hsr_try_limit
from hsr_flexbe_states.hsr_course_demo_state import hsr_CourseDemo
from hsr_flexbe_states.hsr_speech_recognition_state import hsr_SpeechRecognition
from hsr_flexbe_states.hsr_gripping_object_state import hsr_GrippingObject
from hsr_flexbe_states.hsr_follow_me_state import hsr_FollowMe
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jun 15 2019
@author: Hitoshi Nakamura
'''
class course_demo2019SM(Behavior):
	'''
	course_demo2019
	'''


	def __init__(self):
		super(course_demo2019SM, self).__init__()
		self.name = 'course_demo2019'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1111 y:187, x:1077 y:300
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365
		_sm_follow_me_0 = OperatableStateMachine(outcomes=['completed'], input_keys=['tts_text', 'follow_me'])

		with _sm_follow_me_0:
			# x:43 y:153
			OperatableStateMachine.add('hsr_FollowMe',
										hsr_FollowMe(),
										transitions={'continue': 'tts_Followme'},
										autonomy={'continue': Autonomy.Off},
										remapping={'request': 'follow_me'})

			# x:204 y:150
			OperatableStateMachine.add('tts_Followme',
										hsr_TtsInputKey(),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'tts_text'})


		# x:575 y:100, x:568 y:39
		_sm_taketheobject_1 = OperatableStateMachine(outcomes=['continue', 'failed'])

		with _sm_taketheobject_1:
			# x:209 y:40
			OperatableStateMachine.add('tts_AllRight',
										hsr_TtsInputParameter(text="All right.", language="en", delay=2),
										transitions={'completed': 'CloseTheHand'},
										autonomy={'completed': Autonomy.Off})

			# x:385 y:38
			OperatableStateMachine.add('CloseTheHand',
										hsr_GrippingObject(grasp_force=0.7, mode=True),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_success': 'grasp_success'})


		# x:1077 y:108
		_sm_goto_2 = OperatableStateMachine(outcomes=['completed'], input_keys=['move'])

		with _sm_goto_2:
			# x:43 y:31
			OperatableStateMachine.add('SpcoMoveEntrance',
										hsr_SpcofMove(),
										transitions={'move': 'tts_GoTo', 'failed': 'SpcoMoveEntrance'},
										autonomy={'move': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_name': 'move', 'move_text': 'move_text', 'move_pose': 'move_pose', 'arrive_text': 'arrive_text'})

			# x:221 y:31
			OperatableStateMachine.add('tts_GoTo',
										hsr_TtsInputKey(),
										transitions={'completed': 'Move'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'move_text'})

			# x:406 y:29
			OperatableStateMachine.add('Move',
										hsr_MoveBase(),
										transitions={'succeeded': 'tts_Arrived', 'failed': 'Limit_checker'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'move_pose'})

			# x:232 y:161
			OperatableStateMachine.add('tts_FailedMove',
										hsr_TtsInputParameter(text="I failed to make the planning. I will try again.", language="en", delay=2),
										transitions={'completed': 'SpcoMoveEntrance'},
										autonomy={'completed': Autonomy.Off})

			# x:615 y:28
			OperatableStateMachine.add('tts_Arrived',
										hsr_TtsInputKey(),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'arrive_text'})

			# x:480 y:168
			OperatableStateMachine.add('Limit_checker',
										hsr_try_limit(),
										transitions={'give_up': 'tts_GiveupMove', 'try_again': 'tts_FailedMove'},
										autonomy={'give_up': Autonomy.Off, 'try_again': Autonomy.Off})

			# x:634 y:164
			OperatableStateMachine.add('tts_GiveupMove',
										hsr_TtsInputParameter(text="Sorry. I gave up.", language="en", delay=2),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off})


		# x:30 y:365
		_sm_learnspatialconcept_3 = OperatableStateMachine(outcomes=['completed'], input_keys=['training'])

		with _sm_learnspatialconcept_3:
			# x:180 y:40
			OperatableStateMachine.add('hsr_SpcofTraining',
										hsr_SpcofTraining(),
										transitions={'completed': 'tts_Spco_Result'},
										autonomy={'completed': Autonomy.Off},
										remapping={'request': 'training', 'spcof_text': 'spcof_text'})

			# x:30 y:42
			OperatableStateMachine.add('tts_Spco_Result',
										hsr_TtsInputKey(),
										transitions={'completed': 'completed'},
										autonomy={'completed': Autonomy.Off},
										remapping={'tts_text': 'spcof_text'})


		# x:373 y:54
		_sm_start_4 = OperatableStateMachine(outcomes=['completed'])

		with _sm_start_4:
			# x:30 y:43
			OperatableStateMachine.add('hsr_MoveNeutral',
										hsr_MoveNeutral(),
										transitions={'continue': 'completed'},
										autonomy={'continue': Autonomy.Off})



		with _state_machine:
			# x:60 y:42
			OperatableStateMachine.add('Start',
										_sm_start_4,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit})

			# x:442 y:223
			OperatableStateMachine.add('LearnSpatialConcept',
										_sm_learnspatialconcept_3,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit},
										remapping={'training': 'training'})

			# x:464 y:310
			OperatableStateMachine.add('GoTo',
										_sm_goto_2,
										transitions={'completed': 'tts_next_command'},
										autonomy={'completed': Autonomy.Inherit},
										remapping={'move': 'move'})

			# x:885 y:279
			OperatableStateMachine.add('tts_FinishDemonstration',
										hsr_TtsInputParameter(text="本日はオープンキャンパスにご来場いただきありがとうございました。ぜひ立命館に来てくださいね！", language="ja", delay=0),
										transitions={'completed': 'finished'},
										autonomy={'completed': Autonomy.Off})

			# x:228 y:173
			OperatableStateMachine.add('tts_next_command',
										hsr_TtsInputParameter(text="Please give me the next command", language="en", delay=2),
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Off})

			# x:746 y:176
			OperatableStateMachine.add('hsr_CourseDemo',
										hsr_CourseDemo(),
										transitions={'follow_me': 'Follow Me', 'retry_recognition': 'tts_RetryRecognition', 'data': 'tts_RetryRecognition', 'learn': 'LearnSpatialConcept', 'goto': 'GoTo', 'finish': 'tts_FinishDemonstration', 'start_position': 'tts_RetryRecognition', 'take_this': 'TakeTheObject'},
										autonomy={'follow_me': Autonomy.Off, 'retry_recognition': Autonomy.Off, 'data': Autonomy.Off, 'learn': Autonomy.Off, 'goto': Autonomy.Off, 'finish': Autonomy.Off, 'start_position': Autonomy.Off, 'take_this': Autonomy.Off},
										remapping={'recognition': 'recognition', 'follow_me': 'follow_me', 'training': 'training', 'tts_text': 'tts_text', 'move': 'move'})

			# x:55 y:167
			OperatableStateMachine.add('hsr_SpeechRecognition',
										hsr_SpeechRecognition(topic="/julius/speech2text/en"),
										transitions={'recognition': 'hsr_CourseDemo'},
										autonomy={'recognition': Autonomy.Off},
										remapping={'recognition': 'recognition'})

			# x:237 y:417
			OperatableStateMachine.add('hsr_MoveNeutralAgain',
										hsr_MoveNeutral(),
										transitions={'continue': 'hsr_SpeechRecognition'},
										autonomy={'continue': Autonomy.Off})

			# x:463 y:415
			OperatableStateMachine.add('TakeTheObject',
										_sm_taketheobject_1,
										transitions={'continue': 'hsr_MoveNeutralAgain', 'failed': 'hsr_SpeechRecognition'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:446 y:106
			OperatableStateMachine.add('tts_RetryRecognition',
										hsr_TtsInputParameter(text="Could you speak again?", language="en", delay=2),
										transitions={'completed': 'tts_next_command'},
										autonomy={'completed': Autonomy.Off})

			# x:503 y:3
			OperatableStateMachine.add('Follow Me',
										_sm_follow_me_0,
										transitions={'completed': 'hsr_SpeechRecognition'},
										autonomy={'completed': Autonomy.Inherit},
										remapping={'tts_text': 'tts_text', 'follow_me': 'follow_me'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

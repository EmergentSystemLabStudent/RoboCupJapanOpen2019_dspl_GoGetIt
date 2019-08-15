#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_boolean_dummy import hsr_BooleanDummy
from hsr_flexbe_states.hsr_follow_me_state import hsr_FollowMe
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Aug 01 2019
@author: HItoshi Nakamura
'''
class test_followmeSM(Behavior):
	'''
	test_followme
	'''


	def __init__(self):
		super(test_followmeSM, self).__init__()
		self.name = 'test_followme'

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
			# x:77 y:81
			OperatableStateMachine.add('hsr_BooleanDummy',
										hsr_BooleanDummy(text=""),
										transitions={'continue': 'hsr_FollowMe'},
										autonomy={'continue': Autonomy.Off},
										remapping={'follow_me': 'follow_me'})

			# x:252 y:83
			OperatableStateMachine.add('hsr_FollowMe',
										hsr_FollowMe(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'request': 'follow_me'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

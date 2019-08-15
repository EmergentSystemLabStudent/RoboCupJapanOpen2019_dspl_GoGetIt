#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun May 12 2019
@author: Hitoshi Nakamura
'''
class Test_behaviorsSM(Behavior):
	'''
	Sample behavior for studying.
	'''


	def __init__(self):
		super(Test_behaviorsSM, self).__init__()
		self.name = 'Test_behaviors'

		# parameters of this behavior
		self.add_parameter('wait_time', 2)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		hello = "Hello World!!"
		# x:439 y:64, x:130 y:478
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:62 y:47
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'Greeting'},
										autonomy={'done': Autonomy.Off})

			# x:232 y:47
			OperatableStateMachine.add('Greeting',
										LogState(text=hello, severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.High})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

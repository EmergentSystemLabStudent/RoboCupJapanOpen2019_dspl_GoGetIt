#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hsr_flexbe_states.hsr_move_base_client import hsr_MoveBaseClient
from hsr_flexbe_states.hsr_move_base_state import hsr_MoveBase
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Nov 13 2018
@author: shota isobe
'''
class runflexbebehaviorautomaticallySM(Behavior):
	'''
	run flexbe behavior automatically
	'''


	def __init__(self):
		super(runflexbebehaviorautomaticallySM, self).__init__()
		self.name = 'run flexbe behavior automatically'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:7 y:121, x:11 y:204
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:43 y:25
			OperatableStateMachine.add('hsr_MoveBaseClient1',
										hsr_MoveBaseClient(pose_position_x=0.5, pose_position_y=0.0, pose_orientation_z=0.0, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBase1'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose1'})

			# x:227 y:22
			OperatableStateMachine.add('hsr_MoveBase1',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_MoveBaseClient2', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose1'})

			# x:386 y:22
			OperatableStateMachine.add('hsr_MoveBaseClient2',
										hsr_MoveBaseClient(pose_position_x=0.5, pose_position_y=0.5, pose_orientation_z=0.0, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBase2'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose2'})

			# x:565 y:21
			OperatableStateMachine.add('hsr_MoveBase2',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_MoveBaseClient3', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose2'})

			# x:401 y:101
			OperatableStateMachine.add('hsr_MoveBase3',
										hsr_MoveBase(),
										transitions={'succeeded': 'hsr_MoveBaseClient4', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose3'})

			# x:62 y:99
			OperatableStateMachine.add('hsr_MoveBase4',
										hsr_MoveBase(),
										transitions={'succeeded': 'finished', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'request': 'pose4'})

			# x:553 y:112
			OperatableStateMachine.add('hsr_MoveBaseClient3',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.5, pose_orientation_z=0.0, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBase3'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose3'})

			# x:224 y:99
			OperatableStateMachine.add('hsr_MoveBaseClient4',
										hsr_MoveBaseClient(pose_position_x=0.0, pose_position_y=0.0, pose_orientation_z=0.0, pose_orientation_w=1.0),
										transitions={'completed': 'hsr_MoveBase4'},
										autonomy={'completed': Autonomy.Off},
										remapping={'pose': 'pose4'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]

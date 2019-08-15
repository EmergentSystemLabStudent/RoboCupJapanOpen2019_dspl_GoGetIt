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
from flexbe_states.log_key_state import LogKeyState
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
from flexbe_manipulation_states.get_joint_values_state import GetJointValuesState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 05 2018
@author: Lotfi El Hafi
'''
class HSRExampleBehaviorSM(Behavior):
	'''
	This is an example HSR behavior.
	'''


	def __init__(self):
		super(HSRExampleBehaviorSM, self).__init__()
		self.name = 'HSR Example Behavior'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1070 y:60, x:1070 y:209
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.joint_config = [-1.0, 0.5]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:110 y:168
			OperatableStateMachine.add('flexbe_WaitState',
										WaitState(wait_time=1.0),
										transitions={'done': 'flexbe_GetJointValuesState'},
										autonomy={'done': Autonomy.Off})

			# x:516 y:175
			OperatableStateMachine.add('flexbe_LogKeyState',
										LogKeyState(text='joint_values: {}', severity=Logger.REPORT_HINT),
										transitions={'done': 'hsr_MoveitToJointsState'},
										autonomy={'done': Autonomy.Off},
										remapping={'data': 'joint_values'})

			# x:753 y:82
			OperatableStateMachine.add('hsr_MoveitToJointsState',
										MoveitToJointsState(move_group='whole_body', joint_names=['arm_flex_joint', 'arm_lift_joint'], action_topic='/move_group'),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'joint_config'})

			# x:291 y:55
			OperatableStateMachine.add('flexbe_GetJointValuesState',
										GetJointValuesState(joints=['arm_flex_joint', 'arm_lift_joint', 'arm_roll_joint', 'base_l_drive_wheel_joint', 'base_r_drive_wheel_joint', 'base_roll_joint', 'hand_l_spring_proximal_joint', 'hand_motor_joint', 'hand_r_spring_proximal_joint', 'head_pan_joint', 'head_tilt_joint', 'wrist_flex_joint', 'wrist_roll_joint']),
										transitions={'retrieved': 'flexbe_LogKeyState'},
										autonomy={'retrieved': Autonomy.Off},
										remapping={'joint_values': 'joint_values'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]

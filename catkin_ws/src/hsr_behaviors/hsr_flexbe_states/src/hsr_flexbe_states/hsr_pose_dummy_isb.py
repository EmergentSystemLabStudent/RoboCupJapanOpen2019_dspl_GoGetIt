#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
import moveit_commander

class hsr_PoseDummyIsb(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    #> dummy_pose               string     dummy_pose

    #> dummy_object             string     dummy_category

    #> dummy_failed_objects     string     dummy_failed_object

    <= continue        continue
    '''

    def __init__(self):
        super(hsr_PoseDummyIsb,self).__init__(outcomes=['continue'],output_keys=['dummy_pose', 'dummy_object', 'dummy_failed_objects'])
        self._move_group = 'whole_body'
        self.group = moveit_commander.MoveGroupCommander(self._move_group)

    def execute(self, userdata):
        return 'continue'

    def on_enter(self, userdata):
        self.current_pose = self.group.get_current_pose().pose
        #self.current_pose.position.x = self.current_pose.position.x + 1
        #self.current_pose.position.y = self.current_pose.position.y + 0.3
        self.current_pose.position.z = 0.7
        userdata.dummy_pose = {"dummy":self.current_pose}
        userdata.dummy_object = ['toy_airplane', 'doll_pig', 'doll_rabbit', 'sound_maracas', 'sound_keyboard', 'doll_penguin', 'block_star', 'sound_whistle', 'doll_sheep', 'block_cylind_wood', 'toy_car', 'toy_truck', 'doll_monkey', 'block_cube_rits', 'doll_bear']

        userdata.dummy_failed_object = []

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

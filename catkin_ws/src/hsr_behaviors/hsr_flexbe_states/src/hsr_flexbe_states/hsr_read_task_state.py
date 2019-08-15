#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from flexbe_core import EventState, Logger
import moveit_commander
import linecache
import os


class hsr_ReadTask(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.
    ># place_num   int   place number
    #> target_list list  Object list for taget place
    <= continue        continue
    '''

    def __init__(self):
        super(hsr_ReadTask, self).__init__(outcomes=['continue', 'failed'],input_keys=['place_num'], output_keys=['target_list'])
        self.read_file = False

    def execute(self, userdata):
        if self.read_file:
            return 'continue'
        else:
            return 'failed'

    def on_enter(self, userdata):
        file_path = '/root/HSR/catkin_ws/src/em_spco_formation/training_data/default/place2object.txt'
        if os.path.exists(file_path)==1:
            #if the file exits
            line_number = userdata.place_num

            #return the specific line in the txt file
            lines = self.get_line_context(file_path, line_number+1)
            object = lines.split(',')
            userdata.target_list = object
            #get the lines into an array
            self.read_file = True
        else:
            self.read_file = False

    #enter the line number. enter?or there is already a number for use?
    def get_line_context(self, file_path, line_number):
        return(linecache.getline(file_path, line_number).strip())

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger

# example import of required action
import csv

class hsr_AskPlace(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    #> place_name          string  place_name
    #> question            string  question
    #> place_number          int   place_number

    <= continue        continue
    '''

    def __init__(self):
        super(hsr_AskPlace,self).__init__(outcomes=['continue'], output_keys=['place_name', 'question', 'place_number'])
        self.count = 0

    def on_enter(self, userdata):
        csvfile = '/root/HSR/catkin_ws/src/em_spco_formation/training_data/default/word_list.csv'

        f = open(csvfile, "r")
        reader = csv.reader(f)

        base_name = "May I go to the "

        name_list = [row for row in reader]

        userdata.place_name = name_list[self.count][0]
        userdata.question = base_name + name_list[self.count][0] + " ?"
	userdata.place_number = self.count
        self.count += 1

        if self.count == len(name_list):
            self.count = 0

        f.close()

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger

class hsr_SpcofInstruction(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># input    string  input

    #> output   string  ouput

    <= training training
    <= move move
    '''

    def __init__(self):
        super(hsr_SpcofInstruction,self).__init__(outcomes=['training','move'],input_keys=['input'],output_keys=['output'])
        self.sentence = ""

    def execute(self, userdata):
        if "This is" in self.sentence:
            userdata.output = self.sentence.replace("This is ","")
            return 'training'
        elif "learn" in self.sentence:
            userdata.output = "learn"
            return 'training'
        elif "Go to" in self.sentence:
            userdata.output = self.sentence.replace("Go to ","")
            return 'move'

    def on_enter(self, userdata):
        self.sentence = userdata.input
        self.sentence = self.sentence.replace(".","")
        self.sentence = self.sentence.replace("-"," ")
        self.sentence = self.sentence.replace(" the "," ")

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

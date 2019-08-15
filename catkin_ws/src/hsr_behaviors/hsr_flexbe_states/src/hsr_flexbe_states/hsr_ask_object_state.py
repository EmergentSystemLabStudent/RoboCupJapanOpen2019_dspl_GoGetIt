#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger


class hsr_AskObject(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.
    -- repeat               int   repeat
    ># place_number         int   place_number
    ># object_list          list  object_names
    #> object_name          string  object_name
    #> question             string  question

    <= continue        continue
    <= void_object     void_object
    '''

    def __init__(self, repeat=2):
        super(hsr_AskObject,self).__init__(outcomes=['continue', 'void_object'], output_keys=['object_name', 'question'], input_keys=['object_list', 'place_number'])
        # self.count = 0
        self.count_dict = {0:0, 1:0, 2:0}
        self.repeat = repeat
        self.object_name = None
        self.question = None

    def on_enter(self, userdata):
        name_list = userdata.object_list
        place_number = userdata.place_number
        base_name = "May I take the "
        print("name_list_len")
        print(len(name_list))
        print(name_list)
        if self.count_dict[place_number] >= len(name_list)*self.repeat:
            self.question = None
            self.object_name = ""
            print("Finish ask object")
        else:
            print("self.count")
            print(self.count_dict[place_number])
            self.object_name = name_list[self.count_dict[place_number]%len(name_list)]
            self.question = base_name + name_list[self.count_dict[place_number]%len(name_list)] + " ?"
            self.count_dict[place_number] += 1

        # if self.count == len(name_list):
        #     self.count = 0

    def execute(self, userdata):
        userdata.object_name = self.object_name
        userdata.question = self.question
        if self.object_name == "":
            return 'void_object'
        else:
            return 'continue'

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

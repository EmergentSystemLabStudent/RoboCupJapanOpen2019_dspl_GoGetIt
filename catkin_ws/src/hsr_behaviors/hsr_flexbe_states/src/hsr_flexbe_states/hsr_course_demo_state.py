#!/usr/bin/env python
import rospy
from em_follow_me.srv import *
from flexbe_core import EventState, Logger

class hsr_CourseDemo(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># recognition    string  goal name

    #> follow_me     bool    follow_me instruction
    #> training     string  training instruction
    #> tts_text     string  tts text
    #> move         string  move place

    <= retry_recognition     retry_recognition
    <= data             data
    <= learn            learn
    <= goto            goto
    <= finish            finish
    <= start_postion            start_postion
    <= take_this            take_this
    '''

    def __init__(self):
        super(hsr_CourseDemo,self).__init__(outcomes=['follow_me','retry_recognition', 'data', 'learn', 'goto', 'finish', 'start_position', 'take_this'],input_keys=['recognition'],output_keys=['follow_me', 'training', 'tts_text', 'move'])
        self.follow_me_srv = rospy.ServiceProxy('em_follow_me_srv', follow_me)
        self.text = str()
        self.state = 0
        self.follow_me_state = "off"

    def execute(self, userdata):
        if self.text == "Please follow me." and self.follow_me_state == "off":
            self.follow_me_state = "on"
            userdata.follow_me = True
            userdata.tts_text = "I am following you."
            return 'follow_me'
        elif self.text == "Please stop following me." and self.follow_me_state == "on":
            self.follow_me_state = "off"
            userdata.follow_me = False
            userdata.tts_text = " I am no more following you."
            return 'follow_me'
        elif self.text == "This is the StartPosition":
            return 'start_postion'
        elif "This is the " in self.text:
            data = self.text.replace("This is the ","")
            data = data.replace(".","")
            userdata.training = data
            return 'data'
        elif self.text == "Learn the spatial concept.":
            self.state = 1
            if self.follow_me_state == "on":
                try:
                    res = self.follow_me_srv(False)
                    self.follow_me_state = "off"
                except rospy.ServiceException, e:
                    rospy.loginfo("[Service follow_me/client] call failed: %s", e)
            userdata.training = "learn"
            return 'learn'
        elif "Go to the " in self.text and self.state == 1:
            data = self.text.replace("Go to the ","")
            data = data.replace(".","")
            userdata.move = data
            return 'goto'
        elif self.text == "Please take this":
            return 'take_this'
        elif self.text == "Ema Finish the demonstration.":
            return 'finish'
        else:
            return 'retry_recognition'


    def on_enter(self, userdata):
        self.text = userdata.recognition
        self.text = self.text.replace("-"," ")
        self.text = self.text.replace("  "," ")
        #self.text = self.text.replace("newest product","newest_product")
        print self.text

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

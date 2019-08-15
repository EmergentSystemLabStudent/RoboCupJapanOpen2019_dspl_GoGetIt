#!/usr/bin/env python
import rospy
from em_follow_me.srv import *
from flexbe_core import EventState, Logger

class hsr_Final_Demo(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># recognition    string  goal name

    #> follow_me     bool    follow_me instruction
    #> training     string  training instruction
    #> tts_text     string  tts text
    #> move         string  move place

    <= retry_recognition     retry_recognition
    <= data              data
    <= learn             learn
    <= goto              goto
    <= bring_me          bring_me
    <= my_room           my_room
    <= cola              cola
    <= take_cola         take_cola
    <= finish            finish
    '''

    def __init__(self):
        super(hsr_Final_Demo,self).__init__(outcomes=['retry_recognition', 'data', 'learn', 'goto', 'bring_me', 'change_cola', 'change_desk', 'take_cola', 'take_cola_again'],input_keys=['recognition'],output_keys=['follow_me', 'training', 'tts_text', 'move'])
        self.text = str()
        self.state = -1

    def execute(self, userdata):
        if self.text == "This is the cola." and self.state == 4:
            self.state = 5
            return 'change_cola'
        elif self.text == "This is the yoshiki's desk." and self.state == 1:
            self.state = 2
            return 'change_desk'
        elif "This is the " in self.text:
            data = self.text.replace("This is the ","")
            data = data.replace(".","")
            userdata.training = data
            return 'data'
        elif self.text == "Learn the spatial concept." and self.state == -1:
            self.state = 0
            if self.follow_me_state == "on":
                try:
                    res = self.follow_me_srv(False)
                    self.follow_me_state = "off"
                except rospy.ServiceException, e:
                    rospy.loginfo("[Service follow_me/client] call failed: %s", e)
            userdata.training = "learn"
            return 'learn'
        elif "Go to the " in self.text and self.state == 2:
            self.state = 3
            data = self.text.replace("Go to the ","")
            data = data.replace(".","")
            userdata.move = "Desk"
            return 'goto'
        elif self.text == "Bring me a cola from yoshiki's desk." and self.state == 0:
            self.state = 1
            return 'bring_me'
        elif self.text == "Take the cola." and self.state == 3:
            self.state = 4
            return 'take_cola'
        elif self.text == "Take the cola again." and self.state == 5:
            self.state = 6
            return 'take_cola_again'
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

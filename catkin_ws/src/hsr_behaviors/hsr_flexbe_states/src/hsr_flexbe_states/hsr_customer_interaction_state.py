#!/usr/bin/env python
import rospy
from em_follow_me.srv import *
from flexbe_core import EventState, Logger

class hsr_CustomerInteraction(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># recognition    string  goal name
    ># grasp_status string  grasp status

    #> follow_me     bool    follow_me instruction
    #> training     string  training instruction
    #> move         string  move place
    #> object       string  object name
    #> tts_text     string  tts text

    <= follow_me         follow_me
    <= data             data
    <= learn            learn
    <= entrance         entrance
    <= wait             wait
    <= newest_product   newest_product
    <= retry_newest_product   newest_product
    <= toilet           toilet
    <= future1           future1
    <= future2           future2
    <= future3           future3
    <= grasp_fail_talk   fail_talk
    <= success_grasp     success_grasp
    <= retry_recognition     retry_recognition
    '''

    def __init__(self):
        super(hsr_CustomerInteraction,self).__init__(outcomes=['follow_me','data','learn','entrance','wait','newest_product','retry_newest_product','toilet','future1','future2', 'future3', 'grasp_fail_talk', 'success_grasp','retry_recognition'],input_keys=['recognition', 'grasp_status'],output_keys=['follow_me','training','move','object','tts_text'])
        self.follow_me_srv = rospy.ServiceProxy('em_follow_me_srv', follow_me)
        self.text = str()
        self.follow_me_state = "off"
        self.state = 0
        self.plan_failed = 0
        self.grasp_status = str()

    def execute(self, userdata):
        if self.text == "Please follow me." and self.follow_me_state == "off" and self.state == 0:
            self.follow_me_state = "on"
            userdata.follow_me = True
            userdata.tts_text = "I am following you."
            return 'follow_me'
        elif self.text == "Please stop following me." and self.follow_me_state == "on" and self.state == 0:
            self.follow_me_state = "off"
            userdata.follow_me = False
            userdata.tts_text = " I am no more following you."
            return 'follow_me'
        elif "This is the " in self.text and self.state == 0:
            data = self.text.replace("This is the ","")
            data = data.replace(".","")
            userdata.training = data
            return 'data'
        elif self.text == "Please learn the new concepts." and (self.state == 0 or self.state == 1):
            self.state = 1
            if self.follow_me_state == "on":
                try:
                    res = self.follow_me_srv(False)
                    self.follow_me_state = "off"
                except rospy.ServiceException, e:
                    rospy.loginfo("[Service follow_me/client] call failed: %s", e)
            userdata.training = "learn"
            return 'learn'
        elif self.text == "Please go to the entrance." and self.state == 1:
            self.state = 2
            userdata.move = "entrance"
            return 'entrance'
        elif self.text == "Please start serving the customers." and self.state == 2:
            self.state = 3
            return 'wait'
        elif self.state == 3:
            if self.grasp_status == 'not_grasp':
                self.state = 4
                return 'grasp_fail_talk'
            elif self.grasp_status == 'fail_plan' or self.grasp_status == 'not_detect':
                if self.plan_failed > 2:
                    self.state = 4
                    return 'grasp_fail_talk'
                else:
                    self.plan_failed = self.plan_failed + 1
                    userdata.move = "newest_product"
                    userdata.object = ["bottle","remote"]
                    return 'retry_newest_product'
            elif self.grasp_status == 'Success!':
                self.state = 4
                return 'success_grasp'
            elif self.text == "Could you show me the newest_product?":
                userdata.move = "newest_product"
                userdata.object = ["bottle","remote"]
                return 'newest_product'
        elif self.text == "Could you show me the toilet?" and self.state == 4:
            self.state = 5
            userdata.move = "toilet"
            return 'toilet'
        elif self.text == "What else can you do?" and (self.state == 5 or self.state == 2):
            self.state = 6
            return 'future1'
        elif self.text == "I am ready with my smartphone." and self.state == 6:
            self.state = 7
            return 'future2'
        elif self.text == "I would like a digital point card." and self.state == 7:
            self.state = 8
            return 'future3'

        else:
            return 'retry_recognition'


    def on_enter(self, userdata):
        self.text = userdata.recognition
        self.grasp_status = userdata.grasp_status
        self.text = self.text.replace("-"," ")
        self.text = self.text.replace("  "," ")
        self.text = self.text.replace("newest product","newest_product")

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

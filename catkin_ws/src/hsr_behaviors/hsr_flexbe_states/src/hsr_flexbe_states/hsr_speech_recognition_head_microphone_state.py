#!/usr/bin/env python
import rospy, actionlib
from em_led_control.msg import *
from std_msgs.msg import String, ColorRGBA
from tmc_rosjulius_msgs.msg import RecognitionResult
import subprocess
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

class hsr_SpeechRecognitionByHeadMic(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    #> recognition  String  speech recognition

    <= recognition  recognition

    '''

    def __init__(self):
        super(hsr_SpeechRecognitionByHeadMic,self).__init__(outcomes=['recognition'],output_keys=['recognition'])
        self._topic = "/hsrb/voice/text"
        self.Speech2Text_CB = ProxySubscriberCached({self._topic: RecognitionResult})
        #self.led_cli = actionlib.SimpleActionClient('em_led_action', ledAction) # LED
        self.goal = ledGoal()
        p = subprocess.Popen("rosservice call /hsrb/voice/stop_recognition '{}'", shell=True)

    def execute(self, userdata):
        if self.Speech2Text_CB.has_msg(self._topic):
            text = self.Speech2Text_CB.get_last_msg(self._topic)
            result = text.sentences
            rospy.loginfo('[Subscribe %s] "%s"', self._topic, result[0])
            print result[0]
            userdata.recognition = result[0]
            self.Speech2Text_CB.remove_last_msg(self._topic)
            return 'recognition'

    def on_enter(self, userdata):
        #self.led_cli.wait_for_server()
        self.goal.mode = 2
        self.goal.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.0)
        self.goal.flash_interval = 5
        #self.led_cli.send_goal(self.goal)
        p = subprocess.Popen("rosservice call /hsrb/voice/start_recognition '{}'", shell=True)

    def on_exit(self, userdata):
        self.goal.mode = 1
        self.goal.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.0)
        self.goal.flash_interval = 0
        #self.led_cli.send_goal(self.goal)
        p = subprocess.Popen("rosservice call /hsrb/voice/stop_recognition '{}'", shell=True)

    def on_start(self):
        pass

    def on_stop(self):
        pass

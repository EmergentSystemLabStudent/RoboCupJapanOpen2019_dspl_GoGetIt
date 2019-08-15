#!/usr/bin/env python
import rospy
from em_speech.srv import *
from flexbe_core import EventState, Logger

from time import sleep
class hsr_TtsInputKey(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.
    -- delay        float delay

    ># tts_text         string       test

    <= completed    finish speech

    '''

    def __init__(self, delay=1.5):
        super(hsr_TtsInputKey,self).__init__(outcomes=['completed'],input_keys=['tts_text'])
        self.em_speech_srv = rospy.ServiceProxy("em_speech_en", em_speech_en)
        self._delay = delay
    def execute(self, userdata):
        try:
            sleep(self._delay)
            res = self.em_speech_srv(userdata.tts_text)
        except rospy.ServiceException, e:
            rospy.loginfo("[Service speech/client] call failed: %s", e)
        if res.result:
            rospy.loginfo('[Service speech/english] "%s"', userdata.tts_text)
            return 'completed'

    def on_enter(self, userdata):
        rospy.wait_for_service("em_speech_en")

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

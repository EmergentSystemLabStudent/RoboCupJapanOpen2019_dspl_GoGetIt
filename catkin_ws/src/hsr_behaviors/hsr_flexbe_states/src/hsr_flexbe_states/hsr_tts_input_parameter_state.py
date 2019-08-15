#!/usr/bin/env python
import rospy
from em_speech.srv import *
from flexbe_core import EventState, Logger

from time import sleep

class hsr_TtsInputParameter(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- text         string  test
    -- language     string  en or ja

    <= completed    finish speech

    '''

    def __init__(self, text, language, delay):
        super(hsr_TtsInputParameter,self).__init__(outcomes=['completed'])
        self.em_speech_srv_en = rospy.ServiceProxy("em_speech_en", em_speech_en)
        self.em_speech_srv_ja = rospy.ServiceProxy("em_speech_ja", em_speech_ja)
        self._text = text
        self._language = language
        self._delay = delay

    def execute(self, userdata):
        if self._language is "en":
            try:
                sleep(self._delay)
                res = self.em_speech_srv_en(self._text)
            except rospy.ServiceException, e:
                rospy.loginfo("[Service speech/client] call failed: %s", e)
            if res.result:
                rospy.loginfo('[Service speech/english] "%s"', self._text)
                return 'completed'

        elif self._language is "ja":
            try:
                res = self.em_speech_srv_ja(self._text)
            except rospy.ServiceException, e:
                rospy.loginfo("[Service speech/client] call failed: %s", e)
            if res.result:
                rospy.loginfo('[Service speech/japanese] "%s"', self._text)
                return 'completed'

    def on_enter(self, userdata):
        if self._language is "en":
            rospy.wait_for_service("em_speech_en")
        elif self._language is "ja":
            rospy.wait_for_service("em_speech_ja")

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

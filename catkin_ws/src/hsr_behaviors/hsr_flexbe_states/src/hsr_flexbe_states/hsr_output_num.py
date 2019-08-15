#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from std_msgs.msg import Bool

class hsr_OutputNum(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- max_count      Int     Default: 0

    #> count_num      Int

    <= end            end

    '''

    def __init__(self, num=0):
        super(hsr_OutputNum,self).__init__(outcomes=['end'],output_keys=['count_num'])
        self.num = num

    def execute(self, userdata):
        userdata.count_num = self.num
        return 'end'

    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

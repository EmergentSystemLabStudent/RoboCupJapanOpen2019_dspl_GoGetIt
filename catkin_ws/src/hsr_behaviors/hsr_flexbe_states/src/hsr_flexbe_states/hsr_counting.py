#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from std_msgs.msg import Bool

class hsr_Counting(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- max_count      Int     Default: 5

    #> count_num      Int

    <= repetition     repetition

    <= end            end

    '''

    def __init__(self, max_count=5):
        super(hsr_Counting,self).__init__(outcomes=['repetition','end'],output_keys=['count_num'])
        self.max_count = max_count
        self.count = 0
    def execute(self, userdata):
        try:
            if self.count < self.max_count:
                userdata.count_num = self.count
                return 'repetition'
            else:
                Logger.loginfo("Counting process is end")
                return 'end'
        except:
            pass

    def on_enter(self, userdata):
        Logger.loginfo("count: "+str(self.count))
        self.count += 1

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

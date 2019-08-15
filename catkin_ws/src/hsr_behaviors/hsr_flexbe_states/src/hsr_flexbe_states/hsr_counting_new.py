#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from std_msgs.msg import Bool

class hsr_CountingNew(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- max_count      Int     Default: 5

    -- ignore_sw      bool    default:True

    #> count_num      Int

    <= repetition     repetition

    <= end            end

    '''

    def __init__(self, max_count=5, ignore_sw=True):
        super(hsr_CountingNew,self).__init__(outcomes=['ignore_phase','repetition','end'],output_keys=['count_num'])
        self.max_count = max_count
        self.ignore_sw = ignore_sw
        self.count = 0
    def execute(self, userdata):
        if self.ignore_sw == True:
            userdata.count_num = self.count
            return 'ignore_phase'
        else:
            if self.count < self.max_count:
                userdata.count_num = self.count
                return 'repetition'
            else:
                Logger.loginfo("Counting process is end")
                return 'end'


    def on_enter(self, userdata):
        Logger.loginfo("count: "+str(self.count))
        if self.count == 1:
            self.ignore_sw = False
        self.count += 1

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

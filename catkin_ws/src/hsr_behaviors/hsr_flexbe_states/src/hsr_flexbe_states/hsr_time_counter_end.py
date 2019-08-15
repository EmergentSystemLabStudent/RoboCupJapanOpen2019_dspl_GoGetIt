#!/usr/bin/env python
import time
from flexbe_core import EventState, Logger

class hsr_TimeCounterEnd(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- switch         String     Select "s", "p" or "e" (s: Start, p:Print e:End; default: "s")

    ># start_time     Int

    <= completed      completed

    '''

    def __init__(self, switch="s"):
        super(hsr_TimeCounterEnd,self).__init__(outcomes=['completed'],input_keys=['start_time'])
        self.switch = switch
        self.start = None

    def execute(self, userdata):
        return 'completed'

    def on_enter(self, userdata):
        if self.switch == "p" or "e":
            self.start = userdata.start_time
            self.run_time = int(time.time())-self.start
            self.time_conversion(self.run_time)

            if self.switch == "e":
                Logger.loginfo("Clear the time counter")
                self.start = None

        else:
            pass

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def time_conversion(self,run_time):
    	# print the running time
    	if run_time >= 3600:
    		second = int(run_time % 60)
    		minute = run_time / 60
    		hour   = int(minute / 60)
    		minute = int(minute % 60)
            #rospy.loginfo("[Service spacoty/learn] time: %dh%dm%ds", hour, minute, second)
    		Logger.loginfo("Run time: " + str(hour) + "h" + str(minute) + "m" + str(second) + "s")
    	elif run_time >= 60:
    		second = int(run_time % 60)
    		minute = int(run_time / 60)
    		Logger.loginfo("Run time: " + str(minute) + "m" + str(second) + "s")
    	else:
    		second = int(run_time)
    		Logger.loginfo("\nRun time: " + str(second) + "s")

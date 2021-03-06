#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from gazebo_msgs.srv import GetWorldProperties

class hsr_TimeSimStart(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- sw                   Bool     True or False (True: Record start time, default: False)

    #> start_time_sim       Int

    <= continue             continue

    '''

    def __init__(self, sw=False,timer=12):
        super(hsr_TimeSimStart,self).__init__(outcomes=['continue'],output_keys=['start_time_sim'])
        self.sw = sw
        self.start_time = None
        self.get_world_info = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)

    def execute(self, userdata):
        world_info = self.get_world_info()
        if self.sw == True:
            self.start_time = int(world_info.sim_time)
            userdata.start_time_sim = self.start_time
            return 'continue'
        else:
            return 'continue'

    def on_enter(self, userdata):
        if self.sw == True:
            rospy.wait_for_service('/gazebo/get_world_properties')

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def time_conversion(run_time):
        # print the running time
        if run_time >= 3600:
            second = int(run_time % 60)
            minute = run_time / 60
            hour   = int(minute / 60)
            minute = int(minute % 60)
            log_msg = "Simulator Time: " + str(hour) + "h" + str(minute) + "m" + str(second) + "s"
            rospy.loginfo(log_msg)
            Logger.loginfo(log_msg)
        elif run_time >= 60:
            second = int(run_time % 60)
            minute = int(run_time / 60)
            log_msg = "Simulator Time: " + str(minute) + "m" + str(second) + "s"
            rospy.loginfo(log_msg)
            Logger.loginfo(log_msg)
        else:
            second = int(run_time)
            log_msg = "Simulator Time: " + str(second) + "s"
            rospy.loginfo(log_msg)
            Logger.loginfo(log_msg)

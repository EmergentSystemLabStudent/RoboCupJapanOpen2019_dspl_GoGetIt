#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from flexbe_core import EventState,Logger
import os

# no_having_object={'diningtable','chair','shelf','trash','person'}

class hsr_LabelArrangement(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.
    ># detection_result result detection result

    -- path string default='/root/HSR/catkin_ws/src/em_spco_formation/training_data/default/place2object.txt'

    <= continue             Given time has passed.

    <= failed                 Example for a failure outcomes
    '''

    def __init__(self, path='/root/HSR/catkin_ws/src/em_spco_formation/training_data/default/place2object.txt'):
        super(hsr_LabelArrangement,self).__init__(outcomes=['continue','failed'],input_keys=['detection_result'])

        self.detection_result = None
        self.path = path
        # with open(self.path, mode='w') as f:
        #     f.write('')

    def execute(self, userdata):
        return 'continue'

    def on_enter(self, userdata):
        self.detection_result = userdata.detection_result

        # Extract objects that can be carried from all objects.
        candidates = set()
        if self.detection_result is None:
            return 'failed'
        if len(self.detection_result.regions) == 0:
            candidates.add("")
        else:
            for n,region in enumerate(self.detection_result.regions):
                if self.detection_result.names[n][0] != '!':
                    candidates.add(self.detection_result.names[n])
        candidates = list(candidates)
        with open(self.path, mode='a') as f:
            f.write(','.join(candidates))
            f.write('\n')
        print(candidates)

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

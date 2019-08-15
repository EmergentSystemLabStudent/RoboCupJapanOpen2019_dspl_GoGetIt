#!/usr/bin/env python
import rospy, random, os, subprocess
import numpy as np
from geometry_msgs.msg import Pose
from em_spco_tidy_up.srv import *
from flexbe_core import EventState, Logger

class hsr_SelectTidyObjectandPoseRandomlyUltra(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># load_file              string

    ># detect_objects         dict       detect objects

    #> tidy_name_random       list

    #> tidy_pose_random       dict

    <= completed
    '''

    def __init__(self):
        super(hsr_SelectTidyObjectandPoseRandomlyUltra,self).__init__(outcomes=['completed'],input_keys=['load_file','detect_objects'],output_keys=['tidy_name_random','tidy_pose_random'])
        rospy.wait_for_service('em_spco_tidy_up/tidy_pose_randomly')
        self.select_tidy_pose_randomly = rospy.ServiceProxy('em_spco_tidy_up/tidy_pose_randomly', spacoty_object2place)
        self.object_names = []

    def execute(self, userdata):

        ran_idx = random.randint(0,len(self.object_names)-1)
        self.obj_nm = self.object_names.pop(ran_idx)

        req = self.select_tidy_pose_randomly(self.obj_nm)

        if req.done == True:

            userdata.tidy_name_random = [self.obj_nm]

            self.tidy_data.update({self.obj_nm:req.pose})
            userdata.tidy_pose_random = self.tidy_data

            # self.save_obj_pose = np.append(self.save_obj_pose, [self.obj_nm, req.pose.position.x, req.pose.position.y, req.pose.position.z])
            # np.savetxt(self.save_file, self.save_obj_pose, fmt='%s')
            with open(self.save_file, 'a') as f:
                f.write("\n")
                f.write("\t".join([self.obj_nm, str(req.pose.position.x), str(req.pose.position.y), str(req.pose.position.z)]))
            rospy.loginfo("save file: "+self.save_file)
            rospy.loginfo("save file: "+self.save_file)

            return 'completed'


    def on_enter(self, userdata):
        self.save_file = userdata.load_file
        if self.object_names == []:
            self.object_names = userdata.detect_objects.keys()
        self.tidy_data = {}

        self.save_obj_pose = np.loadtxt(self.save_file, dtype=unicode)

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

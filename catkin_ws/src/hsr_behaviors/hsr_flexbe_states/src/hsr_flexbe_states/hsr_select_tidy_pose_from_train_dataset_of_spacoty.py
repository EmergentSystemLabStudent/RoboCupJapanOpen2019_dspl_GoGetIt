#!/usr/bin/env python
import rospy, random, math, os, subprocess
import numpy as np
from geometry_msgs.msg import Pose
from em_spco_tidy_up.srv import *
from flexbe_core import EventState, Logger
import hsrb_interface

class hsr_SelectTidyObjectandPosefromDataset(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># detect_objects         dict     detect objects

    #> tidy_name_random       list

    #> tidy_pose_random       dict

    <= completed
    '''

    def __init__(self):
        super(hsr_SelectTidyObjectandPosefromDataset,self).__init__(outcomes=['completed'],input_keys=['detect_objects'],output_keys=['tidy_name_db','tidy_pose_db'])
        self.robot = hsrb_interface.Robot()
        rospy.wait_for_service('em_spco_tidy_up/tidy_pose_from_dataset')
        self.select_tidy_pose_from_dataset = rospy.ServiceProxy('em_spco_tidy_up/tidy_pose_from_dataset', spacoty_object2place)

    def execute(self, userdata):
        self.tidy_data = {}
        req = self.select_tidy_pose_from_dataset(self.selcted_object_name)

        if req.done == True:

            userdata.tidy_name_db = [self.selcted_object_name]

            self.tidy_data.update({self.selcted_object_name:req.pose})
            userdata.tidy_pose_db = self.tidy_data

            #self.save_obj_pose = np.append(self.save_obj_pose, [self.selcted_object_name, req.pose.position.x, req.pose.position.y, req.pose.position.z])
            # np.savetxt(self.save_file, self.save_obj_pose, fmt='%s')
            # rospy.loginfo("save file: "+self.save_file)

            return 'completed'


    def on_enter(self, userdata):
        # Update self.current_pose.
        whole_body = self.robot.get('whole_body')
        self.current_pose = whole_body.get_end_effector_pose(ref_frame_id='map')
        detect_poses = [poses for poses in userdata.detect_objects.items()]

        if detect_poses != []:
            distance_array = np.array([]) # change HERE for my work.
            for pose in detect_poses:
                distance = \
                    math.sqrt(pow((pose[1].position.x - self.current_pose.pos.x), 2) + pow((pose[1].position.y - self.current_pose.pos.y), 2) + pow((pose[1].position.z - self.current_pose.pos.z), 2))
                distance_array = np.append(distance_array, distance)
            distance_index = np.argsort(distance_array)

            self.selcted_object_name = detect_poses[distance_index[0]][0]

            # self.save_obj_pose = np.loadtxt(self.save_file, dtype=unicode)

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

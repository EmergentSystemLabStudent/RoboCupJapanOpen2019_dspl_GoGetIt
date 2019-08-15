#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
import rospy, rosparam, glob, re
import numpy as np
from scipy.stats import multivariate_normal
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates

class hsr_GetObjectState(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># load_file              string

    -- yolo_yaml              string    load the yaml file for getting labels(default:yolov3.yaml)

    #> object_states          dict

    #> object_states_init     dict

    <= completed              Completed

    <= failed                 Failed

    '''

    def __init__(self, yolo_yaml="yolov3.yaml"):
        super(hsr_GetObjectState,self).__init__(outcomes=['completed','failed'],input_keys=['load_file'], output_keys=['object_states','object_states_init'])
        self._subt_name = "/gazebo/model_states"
        self.sub_get_mdl = ProxySubscriberCached({self._subt_name: ModelStates})

        self._yolo_yaml_path = "/root/HSR/catkin_ws/src/cv_detect_object/scripts/"+yolo_yaml
        obj_yaml = rosparam.load_file(self._yolo_yaml_path)
        self.obj_class = obj_yaml[0][0].values()

        self.mdl_sts = None
        self.count = 0

    def execute(self, userdata):
        self.save_file = userdata.load_file
        if self.mdl_sts == None:
            return 'failed'
        else:
            self.object_info = {}
            for obj_nm in self.mdl_sts.name:
                if obj_nm[4:] in self.obj_class:
                    # pose = Pose()
                    pose = self.mdl_sts.pose[self.mdl_sts.name.index(obj_nm)]
                    self.object_info.update({obj_nm[4:]: pose})

            if self.count == 0:
                self.save_pose()
                userdata.object_states_init = self.object_info
                userdata.object_states = self.object_info
                self.count += 1
            else:
                userdata.object_states = self.object_info
                if self.count % 10 == 0:
                    self.save_pose()
                self.count += 1
            return 'completed'

    def on_enter(self, userdata):
        if self.sub_get_mdl.has_msg(self._subt_name):
            rospy.loginfo("Get States of Object Model.")
            self.mdl_sts = self.sub_get_mdl.get_last_msg(self._subt_name)
            self.sub_get_mdl.remove_last_msg(self._subt_name)
        else:
            rospy.loginfo("Cannnot get states of object model.")

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def save_pose(self):
        with open(self.save_file, 'a') as f:
            for nm, ps in self.object_info.items():
                f.write("\t".join([nm, str(ps.position.x), str(ps.position.y), str(ps.position.z)]))
                f.write("\n")

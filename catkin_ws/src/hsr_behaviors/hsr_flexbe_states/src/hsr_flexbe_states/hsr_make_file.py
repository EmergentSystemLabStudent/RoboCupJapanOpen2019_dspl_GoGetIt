#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
import rospy, rosparam, glob, re, os
import numpy as np
from scipy.stats import multivariate_normal
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates

class hsr_MakeFile(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- save_file     string

    #> load_file     string

    '''

    def __init__(self, save_file="None"):
        super(hsr_MakeFile,self).__init__(outcomes=['completed'],output_keys=['load_file'])
        self.save_file = save_file
        self.sv_f_len = len(save_file)

        self.save_folder = "/root/HSR/catkin_ws/src/em_spco_tidy_up/ex_result/"
        if save_file != "None":
            if os.path.exists(self.save_folder) == False:
                subprocess.Popen("mkdir -p "+self.save_folder, shell=True)

    def execute(self, userdata):
        userdata.load_file = self.save_file
        return 'completed'

    def on_enter(self, userdata):
        rospy.loginfo("save file name: "+self.save_file)
        if self.save_file != "None":
            if self.save_file not in ".csv":
                self.save_file = self.save_file + "_0.csv"
            else:
                self.save_file = self.save_file[:-4] + "_0.csv"
                self.sv_f_len -= 4
            if os.path.exists(self.save_folder) == False:
                subprocess.Popen("mkdir -p "+self.save_folder, shell=True)
                np.savetxt(self.save_folder+self.save_file, np.array([]))
                self.save_file = self.save_folder + self.save_file
            else:
                for i in xrange(100):
                    if os.path.exists(self.save_folder+self.save_file[:self.sv_f_len]+"_"+str(i)+".csv") == False:
                        np.savetxt(self.save_folder+self.save_file[:self.sv_f_len]+"_"+str(i)+".csv", np.array([]))
                        self.save_file = self.save_folder + self.save_file[:self.sv_f_len]+"_"+str(i)+".csv"
                        break
        else:
            self.save_file = None

    def on_exit(self, userdata):
        self.save_file = self.save_file[len(self.save_folder):][:self.sv_f_len]

    def on_start(self):
        pass

    def on_stop(self):
        pass

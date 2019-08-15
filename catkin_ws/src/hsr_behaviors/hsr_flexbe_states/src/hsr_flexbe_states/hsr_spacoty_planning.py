#!/usr/bin/env python
import rospy, os, subprocess
from geometry_msgs.msg import Pose
from em_spco_tidy_up.srv import *
from flexbe_core import EventState, Logger

class hsr_SpaCoTyPlanning(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># load_file          string

    ># detect_objects     dict       detect objects

    #> tidy_pose          dict

    #> tidy_order         list

    #> name2place         bool

    <= completed          move
    '''

    def __init__(self):
        super(hsr_SpaCoTyPlanning,self).__init__(outcomes=['completed'],input_keys=['load_file','detect_objects'],output_keys=['tidy_pose','tidy_order','name2place'])
        rospy.wait_for_service('em_spco_tidy_up/object2place_greedy')
        self.em_spacoty_tidy_greedy = rospy.ServiceProxy('em_spco_tidy_up/object2place_greedy', spacoty_object2place_multi)
        self.tidy_order = []
        self.s2d = {}

    def execute(self, userdata):

        tidy_name = self.tidy_order.pop(0)
        userdata.tidy_pose  = self.s2d
        userdata.tidy_order = [tidy_name]

        if tidy_name in self.low_obj_prob:
            userdata.name2place = True
        else:
            userdata.name2place = False

        return 'completed'


    def on_enter(self, userdata):
        self.save_file = userdata.load_file
        if self.tidy_order == []:
            try:
                dict2string = []
                for k, v in userdata.detect_objects.items():
                    pose_txt = str(v.position.x)+","+ \
                               str(v.position.y)+","+ \
                               str(v.position.z)
                    dict2string.append(k)
                    dict2string.append(pose_txt)
                #print"SpaCoTyPlanning debug: "+str(dict2string)
                req = self.em_spacoty_tidy_greedy(dict2string, self.save_file)
            except rospy.ServiceException, e:
                rospy.loginfo("[Service spacoty/client] call failed: %s", e)
            if req.done == True:
                request_result = req.tidy_object_order_and_pose
                self.low_obj_prob = req.low_obj_prob
                #tidy_order = []
                # string to dict
                self.s2d = {}
                for i in xrange(len(request_result)):
                    if i%2 == 0:
                        p = Pose()
                        p_and_o = request_result[i+1].split(",")
                        p.position.x = float(p_and_o[0])
                        p.position.y = float(p_and_o[1])
                        p.position.z = float(p_and_o[2])
                        #p.orientation.w = 1.0

                        self.s2d.update({request_result[i]: p})
                        self.tidy_order.append(request_result[i])

                str_nm = "["
                for i, nm in enumerate(self.tidy_order):
                    str_nm += " "+str(i)+": "+nm
                str_nm += "]"
                Logger.loginfo("Plan: "+str_nm)
        else:
            str_nm = "["
            for i, nm in enumerate(self.tidy_order):
                str_nm += " "+str(i)+": "+nm
            str_nm += "]"
            Logger.loginfo("Plan: "+str_nm)

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

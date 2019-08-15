#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class hsr_SetObjectState(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># object_name     list

    ># object_pose     dict

    <= completed       Completed

    <= failed          Failed

    '''

    def __init__(self):
        super(hsr_SetObjectState,self).__init__(outcomes=['completed','failed'],input_keys=['object_name','object_pose'])
        self._srv_name  = "/gazebo/set_model_state"
        rospy.wait_for_service(self._srv_name)
        self.srv_set_mdl = rospy.ServiceProxy(self._srv_name, SetModelState)

        self.mdl_sts = None

    def execute(self, userdata):
        if self.mdl_sts == None:
            return 'failed'
        else:
            if "block" in self.obj_nm:
                self.mdl_sts.model_name = "hsr_"+self.obj_nm
            else:
                self.mdl_sts.model_name = "3rd_"+self.obj_nm
            self.mdl_sts.pose = self.obj_ps[self.obj_nm]

            req = self.srv_set_mdl(self.mdl_sts)
            rospy.loginfo(req.status_message)
            if req.success == True:
                return 'completed'
            else:
                return 'failed'

    def on_enter(self, userdata):
        self.obj_nm = userdata.object_name[0]
        self.obj_ps = userdata.object_pose

        if self.obj_nm in self.obj_ps.keys():
            self.mdl_sts = ModelState()


    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

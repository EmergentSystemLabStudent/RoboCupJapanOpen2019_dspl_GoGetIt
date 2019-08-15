#!/usr/bin/env python
import rospy, random
from flexbe_core import EventState, Logger
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class hsr_ResetObjectStateRandomly(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># object_states     Pose

    <= completed         Completed

    <= failed            Failed

    '''

    def __init__(self):
        super(hsr_ResetObjectStateRandomly,self).__init__(outcomes=['completed','failed'],input_keys=['object_states'])
        self._srv_name  = "/gazebo/set_model_state"
        rospy.wait_for_service(self._srv_name)
        self.srv_set_mdl = rospy.ServiceProxy(self._srv_name, SetModelState)

        self.detect_objects = None

    def execute(self, userdata):
        if self.detect_objects == None:
            return 'failed'
        else:
            o_n = self.detect_objects.keys()
            obj_nm = random.sample(o_n,len(o_n))
            obj_ps = random.sample(o_n,len(o_n))
            idx = 0

            for k, v in self.detect_objects.items():
                self.mdl_sts = ModelState()
                if "block" in obj_nm[idx]:
                    self.mdl_sts.model_name = "hsr_"+obj_nm[idx]
                else:
                    self.mdl_sts.model_name = "3rd_"+obj_nm[idx]

                self.mdl_sts.pose.position = self.detect_objects[obj_ps[idx]].position
                self.mdl_sts.pose.orientation = self.detect_objects[obj_nm[idx]].orientation
                idx += 1

                req = self.srv_set_mdl(self.mdl_sts)
                if req.success != True:
                    rospy.loginfo(req.status_message)
                    return 'failed'
                rospy.sleep(0.3)

            rospy.loginfo(req.status_message)
            return 'completed'

    def on_enter(self, userdata):
        rospy.sleep(3.0)
        if userdata.object_states != {}:
            self.detect_objects = userdata.object_states


    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from em_spco_tidy_up.srv import *
from flexbe_core import EventState, Logger

class hsr_SpaCoTyUpdateTidyPose(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># object_name       string     object name

    ># tidy_pose_bef     dict       tidy pose

    #> tidy_pose_aft     dict       tidy pose

    <= completed

    <= failed
    '''

    def __init__(self):
        super(hsr_SpaCoTyUpdateTidyPose,self).__init__(outcomes=['completed','failed'],input_keys=['object_name','tidy_pose_bef'],output_keys=['tidy_pose_aft'])
        rospy.wait_for_service('em_spco_tidy_up/name2place')
        self.em_spacoty_pose = rospy.ServiceProxy('em_spco_tidy_up/name2place', spacoty_name2place)
        self.place_name = None

    def execute(self, userdata):
        try:
            req = self.em_spacoty_pose(self.place_name)
        except rospy.ServiceException, e:
            rospy.loginfo("[Service em_spco_tidy_up/name2place/client] call failed: %s", e)
        if req.done:
            rospy.loginfo("Update Tidy Pose: "+self.object_name)
            self.tidy_pose.update({self.object_name: req.pose})
            userdata.tidy_pose_aft = self.tidy_pose
            return 'completed'
        else:
            return 'failed'

    def on_enter(self, userdata):
        self.object_name = userdata.object_name[0]
        self.tidy_pose = userdata.tidy_pose_bef
        if self.object_name[:4] == "doll":
            self.place_name = "sofa"

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

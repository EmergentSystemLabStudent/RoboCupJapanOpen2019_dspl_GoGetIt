#!/usr/bin/env python
import rospy
from em_spco_tidy_up.srv import *
from flexbe_core import EventState, Logger

class hsr_SpaCoTyTraining(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># save_folder     string    save the dataset to save_folder directory(default:default)

    #> spacoty_text    string    speech text

    <= completed       completed
    '''

    def __init__(self):
        super(hsr_SpaCoTyTraining,self).__init__(outcomes=['completed'],input_keys=['save_folder'],output_keys=['spacoty_text'])
        self.em_spacoty_learn = rospy.ServiceProxy('em_spco_tidy_up/learn', spacoty_learn)
        self.em_spacoty_rviz  = rospy.ServiceProxy('em_spco_tidy_up/rviz',  spacoty_rviz)

        self.sentence = ""

    def execute(self, userdata):
        try:
            teach_count = sum(1 for i in open(self.traindata_path_pose, 'r'))
        except:
            teach_count = 0
        print "teach_count" + str(min(100,teach_count))

        try:
            res = self.em_spacoty_learn(min(100,teach_count))
        except rospy.ServiceException, e:
            rospy.loginfo("[Service spacoty/client] call failed: %s", e)

        if res.finish:
            rospy.loginfo("[Service spacoty/learn] number of concept is %d", res.concept_num)
            try:
                userdata.spacoty_text = "I have finished learning. I can now understand " + str(res.concept_num) + " concepts. I sent the results to my display interface."
                res = self.em_spacoty_rviz(True)
            except rospy.ServiceException, e:
                rospy.loginfo("[Service spacoty/client] call failed: %s", e)
            if res.done:
                rospy.loginfo("[Service spacoty/rviz] publish to rviz")
                return 'completed'

    def on_enter(self, userdata):
        self.traindata_path_pose = "/root/HSR/catkin_ws/src/em_spco_tidy_up/training_data/"+userdata.save_folder+"/pose.csv"
        rospy.wait_for_service('em_spco_tidy_up/learn')
        rospy.wait_for_service('em_spco_tidy_up/rviz')

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

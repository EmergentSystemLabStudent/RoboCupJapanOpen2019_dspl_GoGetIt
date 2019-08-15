#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from flexbe_core import EventState, Logger

class hsr_MoveBase(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># request         Pose         pose

    <= succeeded       finish
    <= failed          error

    '''

    def __init__(self):
        super(hsr_MoveBase,self).__init__(outcomes=['succeeded','failed'],input_keys=['request'])
        self.cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

    def execute(self, userdata):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose = userdata.request
        send_goal = MoveBaseGoal()
        send_goal.target_pose = goal
        self.cli.send_goal(send_goal)
        rospy.loginfo("[Action move_base/move] send goal")
        self.cli.wait_for_result()
        if self.cli.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("[Action move_base/move] succeeded")
            return 'succeeded'
        else:
            rospy.loginfo("[Action move_base/move] failed")
            self.cli.cancel_all_goals()
            return 'failed'

    def on_enter(self, userdata):
        self.cli.wait_for_server()

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

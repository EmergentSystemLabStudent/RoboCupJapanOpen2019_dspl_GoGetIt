#!/usr/bin/env python

import rospy, actionlib

# from em_follow_me.srv import *

# def follow_me_client(hoge):

#     rospy.wait_for_service('em_follow_me_srv')

#     try:
#         example = rospy.ServiceProxy('em_follow_me_srv', follow_me)
#         res = example(hoge)
#         return res.done
#     except rospy.ServiceException, e:
#         print "Service call failed: %s" %e

#     return False

# if __name__ == "__main__":

#     print follow_me_client(True)

#     rospy.sleep(5.0)

#     print follow_me_client(False)

##
from em_follow_me.msg import *

def print_feedback(feedback):
    print feedback
    
if __name__ == '__main__':

    rospy.init_node('follow_me_client')

    client = actionlib.SimpleActionClient('em_follow_me_action', follow_meAction)
    print "start client"
    client.wait_for_server()

    rospy.sleep(10.0)
    goal = follow_meGoal()
    goal.start = True
    client.send_goal(goal, feedback_cb=print_feedback)

    # goal.start = False
    # client.send_goal(goal, feedback_cb=print_feedback)


    client.wait_for_result()
    print client.get_result()
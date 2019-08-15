#! /usr/bin/env python

import rospy
from std_msgs.msg import Bool

from em_follow_me.srv import *

class FollowmeSrv:

    def __init__(self):

        self.flag = False

        self.pub = rospy.Publisher("/em/follow_me", Bool, queue_size=1)
        s = rospy.Service('em_follow_me_srv', follow_me, self.execute)

    def execute(self, goal):

        try:
            self.pub.publish(goal.move)
            self.flag = goal.move
        except Exception as e:
            rospy.loginfo("[em_follow_me_action] %s", e)
            return follow_meResponse(False)

        return follow_meResponse(True)


if __name__ == '__main__':

    rospy.init_node('em_follow_me_action')

    server = FollowmeSrv()
    rospy.spin()
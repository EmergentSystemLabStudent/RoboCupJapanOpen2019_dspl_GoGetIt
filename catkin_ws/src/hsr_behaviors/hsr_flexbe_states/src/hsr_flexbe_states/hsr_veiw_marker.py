#!/usr/bin/env python
from flexbe_core import EventState, Logger

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

class hsr_ViewMarker(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># view_pose     Pose      view_pose

    <= completed             Given time has passed.

    <= failed
    '''

    def __init__(self):
        super(hsr_ViewMarker,self).__init__(outcomes=['continue','failed'],input_keys=['view_pose'])
        self.pub_marker = rospy.Publisher("/moveit/goal_position", Marker, queue_size=1) #ADD

    def execute(self, userdata):
        try:
            self.marker_data.header.frame_id = "map"
            #self.marker_data.stamp = rospy.Time.now()
            #self.marker_data.ns = "sphere"
            #self.marker_data.id = 0
            self.marker_data.action = Marker.ADD

            self.marker_data.pose.position.x = self.pose.position.x
            self.marker_data.pose.position.y = self.pose.position.y
            self.marker_data.pose.position.z = self.pose.position.z

            self.marker_data.pose.orientation.x = 0.0
            self.marker_data.pose.orientation.y = 0.0
            self.marker_data.pose.orientation.z = 0.0
            self.marker_data.pose.orientation.w = 0.0

            self.marker_data.color.r = 1.0
            self.marker_data.color.g = 0.0
            self.marker_data.color.b = 0.0
            self.marker_data.color.a = 0.8

            self.marker_data.scale.x = 0.1
            self.marker_data.scale.y = 0.1
            self.marker_data.scale.z = 0.1

            self.marker_data.lifetime = rospy.Duration(100)

            self.marker_data.type = 2 # sphere

            rospy.sleep(0.3)
            self.pub_marker.publish(self.marker_data)
            rospy.sleep(0.3)
            self.pub_marker.publish(self.marker_data)
            return 'continue'
        except:
            print "Miss the viewing Marker"
            return 'failed'

    def on_enter(self, userdata):
        self.marker_data = Marker()
        self.pose = userdata.view_pose
        print "Get the Moveit Grasping Pose: "+str(self.pose.position)

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
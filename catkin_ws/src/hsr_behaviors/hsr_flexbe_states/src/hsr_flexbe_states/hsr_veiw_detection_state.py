#!/usr/bin/env python
from flexbe_core import EventState, Logger

import rospy
from collections import defaultdict
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose

class hsr_ViewDetection(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.
    ># detection_poses dict          object_poses

    <= completed     Given time has passed.

    '''

    def __init__(self):
        super(hsr_ViewDetection,self).__init__(outcomes=['continue'],input_keys=['object_points'])
        self.pub_marker_array = rospy.Publisher("/yolo_object_coordinate", MarkerArray, queue_size=1) #ADD
        self.id_count = None
        self.object_list = None
        self.marker_array_data = None
    def on_enter(self, userdata):
        self.marker_array_data = MarkerArray()
        self.objects_list = userdata.object_points
        self.id_count = 0
        print "Visualization of Object Points."
        target_poses = [poses for poses in userdata.object_points.items()]
        for pose in target_poses:
            text_marker = self.init_marker()
            text_marker.text = pose[0]
            text_marker.text = text_marker.text.replace('!', '')
            text_marker.pose.position.x = pose[1].position.x
            text_marker.pose.position.y = pose[1].position.y
            text_marker.pose.position.z = pose[1].position.z
            self.id_count += 1
            text_marker.id = self.id_count
            text_marker.ns = "marker"+str(self.id_count)
            self.marker_array_data.markers.append(text_marker)

    def execute(self, userdata):
        rospy.sleep(0.1)
        self.pub_marker_array.publish(self.marker_array_data)
        return 'continue'

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def init_marker(self):
        def_marker = Marker()
        def_marker.type = Marker.TEXT_VIEW_FACING
        def_marker.header.frame_id = "map"
        def_marker.header.stamp = rospy.get_rostime()
        def_marker.action = Marker.ADD
        def_marker.scale.x = 0.15
        def_marker.scale.y = 0.15
        def_marker.scale.z = 0.15
        def_marker.lifetime = rospy.Duration(100)
        def_marker.color.a = 0.9

        return def_marker

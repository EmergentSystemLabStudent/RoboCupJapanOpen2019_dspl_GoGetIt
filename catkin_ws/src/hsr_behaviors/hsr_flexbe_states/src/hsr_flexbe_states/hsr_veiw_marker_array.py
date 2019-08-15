#!/usr/bin/env python
from flexbe_core import EventState, Logger

import rospy
from collections import defaultdict
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose

class hsr_ViewMarkerArray(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># view_pose     dict or Pose    object_points

    <= completed     Given time has passed.

    '''

    def __init__(self):
        super(hsr_ViewMarkerArray,self).__init__(outcomes=['continue'],input_keys=['object_points'])
        self.pub_marker_array = rospy.Publisher("/view_object_points", MarkerArray, queue_size=1) #ADD

    def execute(self, userdata):
        if type(self.objects_list) == dict:
            self.object_names = self.objects_list.keys()
            #self.self.objects_list[obj_nm] = self.objects_list.values()
            Logger.loginfo('Detection Result: %s' %(self.object_names))

            for obj_nm in self.object_names:

                if type(self.objects_list[obj_nm]) == list:
                    for obj_l in self.objects_list[obj_nm]:
                        text_marker, pose_marker = self.init_marker()

                        text_marker.text = obj_nm
                        text_marker.pose.position.x = obj_l.position.x - 0.1
                        text_marker.pose.position.y = obj_l.position.y
                        text_marker.pose.position.z = obj_l.position.z + 0.1
                        text_marker.color.r = min(abs(text_marker.pose.position.x / 2), 1.0)
                        text_marker.color.g = min(abs(text_marker.pose.position.y / 2), 1.0)
                        text_marker.color.b = min(abs(text_marker.pose.position.z / 2), 1.0)

                        pose_marker.pose.position.x = obj_l.position.x
                        pose_marker.pose.position.y = obj_l.position.y
                        pose_marker.pose.position.z = obj_l.position.z
                        pose_marker.color.r = min(abs(pose_marker.pose.position.x / 2), 1.0)
                        pose_marker.color.g = min(abs(pose_marker.pose.position.y / 2), 1.0)
                        pose_marker.color.b = min(abs(pose_marker.pose.position.z / 2), 1.0)

                        self.id_count += 1
                        text_marker.id = self.id_count
                        text_marker.ns = "marker"+str(self.id_count)
                        self.marker_array_data.markers.append(text_marker)
                        self.id_count += 1
                        pose_marker.id = self.id_count
                        pose_marker.ns = "marker"+str(self.id_count)
                        self.marker_array_data.markers.append(pose_marker)
                else:
                    text_marker, pose_marker = self.init_marker()
                    text_marker.text = obj_nm
                    text_marker.pose.position.x = self.objects_list[obj_nm].position.x - 0.1
                    text_marker.pose.position.y = self.objects_list[obj_nm].position.y
                    text_marker.pose.position.z = self.objects_list[obj_nm].position.z + 0.1
                    text_marker.color.r = min(abs(text_marker.pose.position.x / 2), 1.0)
                    text_marker.color.g = min(abs(text_marker.pose.position.y / 2), 1.0)
                    text_marker.color.b = min(abs(text_marker.pose.position.z / 2), 1.0)

                    pose_marker.pose.position.x = self.objects_list[obj_nm].position.x
                    pose_marker.pose.position.y = self.objects_list[obj_nm].position.y
                    pose_marker.pose.position.z = self.objects_list[obj_nm].position.z
                    pose_marker.color.r = min(abs(pose_marker.pose.position.x / 2), 1.0)
                    pose_marker.color.g = min(abs(pose_marker.pose.position.y / 2), 1.0)
                    pose_marker.color.b = min(abs(pose_marker.pose.position.z / 2), 1.0)

                    self.id_count += 1
                    text_marker.id = self.id_count
                    text_marker.ns = "marker"+str(self.id_count)
                    self.marker_array_data.markers.append(text_marker)
                    self.id_count += 1
                    text_marker.id = self.id_count
                    pose_marker.ns = "marker"+str(self.id_count)
                    self.marker_array_data.markers.append(pose_marker)


        elif type(self.objects_list) == Pose:

            for obj_ps in self.objects_list:
                pose_marker = Marker()
                pose_marker.type = 2 # sphere
                pose_marker.header.frame_id = "map"
                pose_marker.header.stamp = rospy.get_rostime()
                pose_marker.id = self.id_count
                pose_marker.action = Marker.ADD

                pose_marker.pose.position.x = obj_ps.position.x
                pose_marker.pose.position.y = obj_ps.position.y
                pose_marker.pose.position.z = obj_ps.position.z

                pose_marker.pose.orientation.x = 0.0
                pose_marker.pose.orientation.y = 0.0
                pose_marker.pose.orientation.z = 0.0
                pose_marker.pose.orientation.w = 0.0

                pose_marker.color.r = min(abs(pose_marker.pose.position.x / 2), 1.0)
                pose_marker.color.g = min(abs(pose_marker.pose.position.y / 2), 1.0)
                pose_marker.color.b = min(abs(pose_marker.pose.position.z / 2), 1.0)
                pose_marker.color.a = 0.8

                pose_marker.scale.x = 0.07
                pose_marker.scale.y = 0.07
                pose_marker.scale.z = 0.07

                pose_marker.lifetime = rospy.Duration(100)

                self.id_count += 1
                pose_marker.ns = "marker"+str(self.id_count)
                self.marker_array_data.markers.append(pose_marker)

        #print self.marker_array_data

        rospy.sleep(0.1)
        self.pub_marker_array.publish(self.marker_array_data)
        rospy.sleep(0.1)
        self.pub_marker_array.publish(self.marker_array_data)
        return 'continue'

    def on_enter(self, userdata):
        self.marker_array_data = MarkerArray()
        self.objects_list = userdata.object_points
        self.id_count = 0
        print "Visualization of Object Points."

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def init_marker(self):
        def_text_marker = Marker()
        def_text_marker.type = Marker.TEXT_VIEW_FACING
        def_text_marker.header.frame_id = "map"
        def_text_marker.header.stamp = rospy.get_rostime()
        def_text_marker.action = Marker.ADD
        def_text_marker.scale.x = 0.15
        def_text_marker.scale.y = 0.15
        def_text_marker.scale.z = 0.15
        def_text_marker.lifetime = rospy.Duration(100)
        def_text_marker.color.a = 0.9

        def_pose_marker = Marker()
        def_pose_marker.type = 2 # sphere
        def_pose_marker.header.frame_id = "map"
        def_pose_marker.header.stamp = rospy.get_rostime()
        def_pose_marker.action = Marker.ADD
        def_pose_marker.scale.x = 0.07
        def_pose_marker.scale.y = 0.07
        def_pose_marker.scale.z = 0.07
        def_pose_marker.lifetime = rospy.Duration(100)
        def_pose_marker.color.a = 0.8

        def_pose_marker.pose.orientation.x = 0.0
        def_pose_marker.pose.orientation.y = 0.0
        def_pose_marker.pose.orientation.z = 0.0
        def_pose_marker.pose.orientation.w = 0.0

        return def_text_marker, def_pose_marker

#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
import sys
import rospy
import cv2
import numpy as np
import cv_bridge
import sensor_msgs.msg
import hsr_common.msg
from timeit import default_timer as timer


class hsr_ImageCapture(EventState):
    '''
    This state is to capture rgb/depth images and get camera_info. Output is images and camera_info.

    -- rgb_topic     string    rgb_topic is "/hsrb/head_rgbd_sensor/rgb/image_rect_color"

    -- depth_topic   string    depth_topic is "/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw"

    -- camera_info_topic string camera_info_topic is "/hsrb/head_rgbd_sensor/rgb/camera_info"

    #> rgb_image           rgb_image    rgb_image obtained

    #> depth_image         depth_image  depth_image obtained

    #> camera_info         camera_info  camera_info obtained

    <= completed                    transition leading to completion

    <= failed                       transition leading to failure

    '''

    def __init__(self, rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info"):
        super(hsr_ImageCapture, self).__init__(
            outcomes=['completed', 'failed'], output_keys=['rgb_image', 'depth_image', 'camera_info'])

        self._depth_topic = depth_topic
        self._rgb_topic = rgb_topic
        self._camera_info_topic = camera_info_topic

        # Retrieved messages
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None

        # Images converted for cv2
        self.cv_rgb_image = None
        self.cv_depth_image = None

        # CvBridge object
        self.bridge = cv_bridge.CvBridge()
        Logger.loginfo("Start!")
        # It may happen that the action client fails to send the action goal.
        self._error = False


    def execute(self, userdata):
        if self._error:
            return 'command_error'

        # timeout [sec]
        timeout = 5.0

        try:
            rgb_msg = rospy.wait_for_message(
                self._rgb_topic, sensor_msgs.msg.Image,
                timeout = timeout
            )
            depth_msg = rospy.wait_for_message(
                self._depth_topic, sensor_msgs.msg.Image,
                timeout = timeout
            )
            camera_info_msg = rospy.wait_for_message(
                self._camera_info_topic, sensor_msgs.msg.CameraInfo,
                timeout = timeout
            )
            cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

            userdata.rgb_image = cv_rgb_image
            userdata.depth_image = cv_depth_image
            userdata.camera_info = camera_info_msg
            result = 'completed'
        except Exception as e:
            rospy.logerr('An error occurred when retrieving images.')
            rospy.logerr(type(e))
            rospy.logerr(e.args)
            rospy.logerr(e)
            result = None
        return result

    def on_enter(self, userdata):
        Logger.loginfo("Waiting image......")
        pass

    def on_exit(self, userdata):
        pass

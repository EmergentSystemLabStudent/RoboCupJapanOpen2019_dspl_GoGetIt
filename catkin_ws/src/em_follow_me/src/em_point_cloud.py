#!/usr/bin/python

import rospy, tf2_ros, tf
import numpy as np
import math, time
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseStamped
import hsrb_interface

sensor_deg_width = 58.0
sensor_deg_height = 45.0
image_height = 480
image_width = 640
width_pixel_deg = sensor_deg_width / image_width
height_pixel_deg = sensor_deg_height / image_height

class FollowmePCL(object):

    def depthImageCallback(self,data):

        time_start = time.time()
        
        # neck_rad = self.whole_body.joint_positions['head_tilt_joint']
        # if neck_rad != -np.radians(45.0):
        #     self.whole_body.move_to_joint_positions({'head_tilt_joint':-np.radians(45.0)})

        try:
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            trans = tf_buffer.lookup_transform("map","head_rgbd_sensor_rgb_frame",rospy.Time(0),rospy.Duration(1.0))
        except Exception as e:
            rospy.loginfo("%s", e)
            return

        depthImage = CvBridge().imgmsg_to_cv2(data, '16UC1')
        heightImage = trans.transform.translation.z - np.array(depthImage)*np.sin(np.radians(45.0))*0.001 - self.floor_adjust
        heightImage[depthImage<10.0] = 0.0

        if self.use_color:
            colorImage = CvBridge().imgmsg_to_cv2(self.colorImageData, 'rgb8')
            colorImage[heightImage<self.heightThreshold] = [0,0,0]
            grayImage = cv2.cvtColor(colorImage, cv2.COLOR_BGR2GRAY)
            retval,bw = cv2.threshold(grayImage,50,225,cv2.THRESH_BINARY|cv2.THRESH_OTSU)
        else:
            depthImage[heightImage<self.heightThreshold] = 0.0
            grayImage = np.uint8(depthImage)
            retval,bw = cv2.threshold(grayImage,1,225,cv2.THRESH_BINARY)

        image, contours, hierarchy = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        # s = cv2.cvtColor(colorImage, cv2.COLOR_BGR2RGBA)
        # s[heightImage<self.heightThreshold] = [0,0,0,0]
        # cv2.imwrite("hoge.png", s)

        print "depth time : ", time.time() - time_start

        br = tf.TransformBroadcaster()
        xywh = np.array(
            [cv2.boundingRect(c) for c in contours if 200 < cv2.contourArea(c) < 2000 and len(c) > 0]
            )
        if len(xywh) < 1:
            return
        if self.use_color:
            for x, y, w, h in xywh:
                cv2.rectangle(colorImage,(x,y),(x+w,y+h),(0,255,0),2)
        elif self.output_flag:
            for x, y, w, h in xywh:
                cv2.rectangle(depthImage,(x,y),(x+w,y+h),255,2)
        x_pixel_uncheck = xywh[:,0] + xywh[:,2] / 2
        y_pixel_uncheck = xywh[:,1] + xywh[:,3] / 2
        dist_uncheck = depthImage[y_pixel_uncheck, x_pixel_uncheck] * 0.001

        check_array = (0.8 < dist_uncheck)
        x_pixel = x_pixel_uncheck[check_array]
        y_pixel = y_pixel_uncheck[check_array]
        dist = dist_uncheck[check_array]

        x_deg = width_pixel_deg * (x_pixel - image_width / 2)
        y_deg = height_pixel_deg * (y_pixel - image_height / 2)
        tf_x = np.sin(np.radians(x_deg)) * dist
        tf_y = np.cos(np.radians(y_deg)) * dist

        for count, (tf_xi, tf_yi) in enumerate(zip(tf_x, tf_y)):
            name = "followeme_" + str(count)
            br.sendTransform((tf_xi, 0.0, tf_yi), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), name, "head_rgbd_sensor_rgb_frame")

        print "fin time : ", time.time() - time_start

        if self.use_color:
            self.image_pub.publish(CvBridge().cv2_to_imgmsg(colorImage, "rgb8"))
        elif self.output_flag:
            self.image_pub.publish(CvBridge().cv2_to_imgmsg(depthImage, "16UC1"))

    def colorImageCallback(self,data):

        self.colorImageData = data


    def __init__(self):

        self.robot      = hsrb_interface.Robot()
        self.whole_body = self.robot.get('whole_body')

        self.whole_body.move_to_joint_positions({'arm_flex_joint':0.0,'arm_lift_joint':0.0,'arm_roll_joint':-1.57,'head_pan_joint':0.0,
            'head_tilt_joint':-np.radians(45.0),'wrist_flex_joint':-1.57,'wrist_roll_joint':0.0})

        self.use_color = rospy.get_param('~use_color')
        self.output_flag = rospy.get_param('~output_flag')
        self.heightThreshold = rospy.get_param('~heightThreshold')

        if self.use_color:
            rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw", Image, self.colorImageCallback, queue_size=1)
        rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_raw", Image, self.depthImageCallback, queue_size=1)

        self.image_pub = rospy.Publisher("/depth_threshold_image/image", Image, queue_size=1)

        deg_adjust = np.loadtxt("deg45.csv", delimiter=",")
        self.floor_adjust = np.array([np.ones(image_width)*deg_adjust[i] for i in xrange(image_height)])

        # self.image_rad_adjust = np.array([np.ones(image_width)*sensor_deg_height/image_height*np.pi/180*(image_height/2 - i - 0.5) for i in xrange(image_height)])


if __name__ == '__main__':

    rospy.init_node('FollowmePCL', anonymous=True)
    hoge = FollowmePCL()
    rospy.spin()

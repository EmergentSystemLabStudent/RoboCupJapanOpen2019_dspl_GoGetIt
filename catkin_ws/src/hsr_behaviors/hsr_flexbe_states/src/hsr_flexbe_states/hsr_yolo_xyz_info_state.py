from flexbe_core import EventState, Logger

import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2

class hsr_YoloXyzInfo(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    <= completed    completed

    '''

    def __init__(self):
        super(hsr_YoloXyzInfo,self).__init__(outcomes=['completed'])
        self.publisher = rospy.Publisher("/darknet_ros/object_coordinate",MarkerArray, queue_size=1)
        self.marker_array = MarkerArray()
        self.finish = False

    def execute(self, userdata):
        while self.finish:
            continue
        return "completed"
    def on_enter(self, userdata):
        self.subscriber = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_callback, queue_size=1)
        rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image_raw',Image,self.depth_callback, queue_size=1)

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def bounding_callback(self, message):
        self.subscriber.unregister()

        for box in message.bounding_boxes:
            cx = (int(box.xmin) + (int(box.xmax)-int(box.xmin))/2)
            cy = (int(box.ymin) + (int(box.ymax)-int(box.ymin))/2)

            depth = self.get_pixel_depth(int(cx),int(cy))

            x = ((cx-320.0) * 554.0 / 320.0) *(depth/1000.0)
            y = -((cy-240.0) * 414.0 / 240.0) *(depth/1000.0)

            if depth == 0 or x == 0 or y == 0:
                continue

            new_object = Marker()
            new_object.pose.position.x = depth / 1000.
            new_object.pose.position.y = - x / 1000.
            new_object.pose.position.z = y / 1000.
            new_object.text = box.Class
            self.marker_array.markers.append(new_object)
            rospy.loginfo(new_object.text)
            rospy.loginfo(new_object.pose.position)
        self.publisher.publish(self.marker_array)
        self.finish = True

    def depth_callback(self, image):
        global global_i
        global_i = image

    def get_pixel_depth(self, x, y):
        cvbridge = CvBridge()
        try:
            img = cvbridge.imgmsg_to_cv2(global_i , "passthrough")
            return img[y][x]
        except:
            img = 1
            return img

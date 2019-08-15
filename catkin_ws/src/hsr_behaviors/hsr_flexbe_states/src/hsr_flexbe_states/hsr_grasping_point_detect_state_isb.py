#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
import sys, rospy, cv2, cv_bridge, os
import numpy as np
from image_geometry import PinholeCameraModel
from timeit import default_timer as timer
from geometry_msgs.msg import Pose
import sensor_msgs.msg
import hsr_common.msg

class hsr_GraspingPointDetectIsb(EventState):
    '''
    This state is to detect bounding box and grasping points for objects in image. Output is detect result.

    -- output_topic      string         output_topic is "/bounding_box_2d_monitor"

    -- save              bool           option saving image(default:True)

    ># rgb_image         rgb_image      rgb_image obtained with other topics

    ># depth_image       depth_image    depth_image obtained with other topics

    ># camera_info       camera_info    camera_info obtained with other topics

    #> grasping_point    dict           grasping_point

    <= completed         transition leading to completion

    <= failed            transition leading to failure

    <= yolo_failed

    '''

    def __init__(self, output_topic="/bounding_box_2d_monitor", save=True):
        super(hsr_GraspingPointDetectIsb, self).__init__(outcomes=['completed', 'failed', 'yolo_failed'], input_keys=['rgb_image', 'depth_image', 'camera_info'],
                                                output_keys=['grasping_point'])
        # Create the action client when building the behavior.
        # This will cause the behavior to wait for the client before starting execution
        # and will trigger a timeout error if it is not available.
        # Using the proxy client provides asynchronous access to the result and status
        # and makes sure only one client is used, no matter how often this state is used in a behavior.
        self._topic = "bounding_box_2d"
        self._output_topic = output_topic
        self._save = save
        self.camera_info = None
        self.rgb_image = None
        self.depth_image = None

        # pass required clients as dict (topic: type)
        self._client = ProxyActionClient(
            {self._topic: hsr_common.msg.BoundingBox2DAction})
        self.bridge = cv_bridge.CvBridge()

        # It may happen that the action client fails to send the action goal.
        self._error = False
        self._get_image_error = False

    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.

        # Check if the client failed to send the goal.
        if self._get_image_error:
            return 'failed'
        elif self._error:
            return 'failed'

        rospy.sleep(0.2)
        # Check if the action has been finished
        if self._client.has_result(self._topic):
            Logger.loginfo("Success for getting the result of YOLO detecttion.")
            result = self._client.get_result(self._topic)
            self.end = timer()
        else:
            return 'yolo_failed'

        if len(result.regions) == 0:
            return 'failed'

        # In this example, we also provide the amount of cleaned dishes as output key.
        # userdata.detection = result
        dict = {}
        for n, region in enumerate(result.regions):
            hsv = cv2.cvtColor(
                np.uint8([[[120*(1-result.scores[n]), 255, 255]]]), cv2.COLOR_HSV2BGR)[0][0]
            x0 = region.x_offset
            y0 = region.y_offset
            x1 = region.x_offset + region.width - 1
            y1 = region.y_offset + region.height - 1
            cv2.rectangle(self.rgb_image, (x0, y0), (x1, y1),
                          (int(hsv[0]), int(hsv[1]), int(hsv[2])), 2)
            label_str = '%.2f: %s' % (result.scores[n], result.names[n])
            text_config = {
                'text': label_str,
                'fontFace': cv2.FONT_HERSHEY_PLAIN,
                'fontScale': 1,
                'thickness': 1,
            }
            size, baseline = cv2.getTextSize(**text_config)
            cv2.rectangle(
                self.rgb_image, (x0, y0), (x0 + size[0], y0 + size[1]),
                (255, 255, 255), cv2.FILLED
            )
            cv2.putText(
                self.rgb_image,
                org=(x0, y0 + size[1]),
                color=(255, 0, 0),
                **text_config
            )
            combinations = [(x, y)
                            for x in range(int(x0 + region.width*0.45 - 1), int(x0 + region.width*0.55 - 1))
                            for y in range(int(y0 + region.height*0.45 - 1), int(y0 + region.height*0.55 - 1))]
            grasping_z = 0
            pixel_count = 0
            for x, y in combinations:
                if (self.depth_image.item(y, x) == 0) or (self.depth_image.item(y, x) == 66535):
                    pass
                else:
                    grasping_z = grasping_z + self.depth_image.item(y, x)
                    pixel_count = pixel_count + 1
            if pixel_count != 0:
                grasping_z = grasping_z / pixel_count
                cx, cy, cz = self.get_direction(
                    self.camera_info, (x0+x1)/2, (y0+y1)/2)
                pose = Pose()
                pose.position.x = cx * grasping_z/1000.0
                pose.position.y = cy * grasping_z/1000.0
                pose.position.z = cz * grasping_z/1000.0
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 0.0

                #dict.setdefault(result.names[n],[]).append(pose)
                dict.update({result.names[n]: pose})

                cv2.rectangle(
                    self.rgb_image, (int(x0 + region.width*0.45 - 1), int(y0 + region.height*0.45 - 1)
                                     ), (int(x0 + region.width*0.55 - 1), int(y0 + region.height*0.55 - 1)),
                    (grasping_z/10000*255, grasping_z/10000 *
                     255, grasping_z/10000*255), cv2.FILLED
                )

        userdata.grasping_point = dict
        #cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)
        try:
            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(self.rgb_image, "bgr8")
            )
            if self._save:
                from datetime import datetime
                img_name = datetime.now().strftime("%Y%m%H%M%S")
                cv2.imwrite(img_name+'.png', self.rgb_image)
                Logger.loginfo('Save the image: %s.png' %(img_name))
        except cv_bridge.CvBridgeError as e:
            Logger.logwarn(str(e))

        # Based on the result, decide which outcome to trigger.
        if not result is None:
            Logger.loginfo('Finished detection. (%f [sec])' % (
                self.end - self.start, ))
            return 'completed'
        else:
            return 'failed'

        # If the action has not yet finished, no outcome will be returned and the state stays active.

    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        # As documented above, we get the specification of which dishwasher to use as input key.
        # This enables a previous state to make this decision during runtime and provide the ID as its own output key.
        self.image_pub = rospy.Publisher(
            self._output_topic, sensor_msgs.msg.Image, latch=True, queue_size=10)

        self.rgb_image = userdata.rgb_image
        self.depth_image = userdata.depth_image
        self.camera_info = userdata.camera_info
        # Create the goal.
        Logger.loginfo("Waiting image......")
        if self.rgb_image is not None:
            Logger.loginfo("Get rgb image!")
            sys.stdout.flush()
        else:
            Logger.logwarn(
                'Failed to get RGB image!')
            self._get_image_error = True
        if self.depth_image is not None:
            Logger.loginfo("Get depth image!")
            sys.stdout.flush()
        else:
            Logger.logwarn(
                'Failed to get Depth image!')
            self._get_image_error = True
        if self.camera_info is not None:
            Logger.loginfo("Get camera info!")
            sys.stdout.flush()
        else:
            Logger.logwarn(
                'Failed to get Camera info!')
            self._get_image_error = True

        self.start = timer()
        goal = hsr_common.msg.BoundingBox2DGoal()
        goal.image = self.bridge.cv2_to_imgmsg(self.rgb_image, "bgr8")

        # Send the goal.
        # make sure to reset the error state since a previous state execution might have failed
        self._error = False
        try:
            self._client.send_goal(self._topic, goal)
        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn(
                'Failed to send the BoundingBox2D command:\n%s' % str(e))
            self._error = True

    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')

    def get_direction(self, camera_info, px, py):
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)
        cx, cy, cz = camera_model.projectPixelTo3dRay((px, py))
        return cx, cy, cz

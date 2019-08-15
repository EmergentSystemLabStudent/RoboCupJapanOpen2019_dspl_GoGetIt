#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached

# example import of required action
import rospy, rosparam, cv2, cv_bridge, os, subprocess, tf2_ros, tf2_geometry_msgs#, sys
import numpy as np
from image_geometry import PinholeCameraModel
from timeit import default_timer as timer
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, String
import sensor_msgs.msg
import hsr_common.msg

class hsr_SpaCoTyGetDataset(EventState):
    '''
    This state is to detect bounding box and grasping points for objects in image. Output is detect result.

    -- output_topic         string    output_topic is "/bounding_box_2d_monitor"

    -- rgb_topic            string    rgb_topic is "/hsrb/head_rgbd_sensor/rgb/image_rect_color"

    -- depth_topic          string    depth_topic is "/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw"

    -- camera_info_topic    string    camera_info_topic is "/hsrb/head_rgbd_sensor/rgb/camera_info"

    -- yolo_yaml            string    load the yaml file for getting labels(default:yolov3.yaml)

    -- save_image           bool      option saving image(default:True)

    -- z_offset             float     offset about z coordinate (default:0.1 [m])

    -- MEMO                 Please push "/spacoty/get_dataset"(False), if u wanna finished

    ># save_folder          string    save the dataset to save_folder directory(default:default)

    <= completed            transition leading to completion

    <= failed               transition leading to failure

    '''

    def __init__(self, output_topic="/bounding_box_2d_monitor", rgb_topic="/hsrb/head_rgbd_sensor/rgb/image_rect_color", depth_topic="/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", camera_info_topic="/hsrb/head_rgbd_sensor/rgb/camera_info", yolo_yaml="yolov3.yaml", save_image=True, z_offset=0.1, MEMO="/spacoty/get_dataset"):
        super(hsr_SpaCoTyGetDataset, self).__init__(outcomes=['completed', 'failed'],input_keys=['save_folder'])
        # Create the action client when building the behavior.
        # This will cause the behavior to wait for the client before starting execution
        # and will trigger a timeout error if it is not available.
        # Using the proxy client provides asynchronous access to the result and status
        # and makes sure only one client is used, no matter how often this state is used in a behavior.
        self._topic = {"img":"bounding_box_2d","sw":"/spacoty/get_dataset","wrd":"/spacoty/place_name"}
        self._output_topic = output_topic
        self._depth_topic = depth_topic
        self._rgb_topic = rgb_topic
        self._camera_info_topic = camera_info_topic

        self._save_folder = "/root/HSR/catkin_ws/src/em_spco_tidy_up/training_data/"
        self._save_image = save_image
        self.camera_info = None
        self.rgb_image = None
        self.depth_image = None

        self.z_offset = z_offset

        # get object labels
        # self._yolo_yaml_path = "/root/HSR/catkin_ws/src/hsr_launch/config/darknet_ros/"+yolo_yaml
        # obj_yaml = rosparam.load_file(self._yolo_yaml_path)
        # self.obj_class = obj_yaml[0][0]["yolo_model"]["detection_classes"]["names"]
        self._yolo_yaml_path = "/root/HSR/catkin_ws/src/cv_detect_object/scripts/"+yolo_yaml
        obj_yaml = rosparam.load_file(self._yolo_yaml_path)
        self.obj_class = obj_yaml[0][0].values()

        # pass required clients as dict (topic: type)
        self._client = ProxyActionClient(
            {self._topic["img"]: hsr_common.msg.BoundingBox2DAction})
        self.bridge = cv_bridge.CvBridge()

        # Subscriber
        self.sub_sw = ProxySubscriberCached({self._topic["sw"]: Bool})
        self.sub_word = ProxySubscriberCached({self._topic["wrd"]: String})

        # define the dimension
        self._data_dim = {"pose":3,"object":len(self.obj_class),"word":0}

        # It may happen that the action client fails to send the action goal.
        self._error = False
        self._get_image_error = False

    def execute(self, userdata):
        # While this state is active, check if the action has been finished and evaluate the result.
        if self.sub_sw.has_msg(self._topic["sw"]):
            Logger.loginfo('Get the Switcher.')
            self.finish_sw = self.sub_sw.get_last_msg(self._topic["sw"])
            self.finish_sw = self.finish_sw.data
            self.sub_sw.remove_last_msg(self._topic["sw"])

        # Get image
        #timeout = 1.0
        rgb_msg = rospy.wait_for_message(
            self._rgb_topic, sensor_msgs.msg.Image#, timeout = timeout
        )
        depth_msg = rospy.wait_for_message(
            self._depth_topic, sensor_msgs.msg.Image#, timeout = timeout
        )
        camera_info_msg = rospy.wait_for_message(
            self._camera_info_topic, sensor_msgs.msg.CameraInfo#, timeout = timeout
        )

        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        self.camera_info = camera_info_msg
        # Logger.loginfo("Got the image...")

        if self.finish_sw == True and self._ignore == False:
            # Create the goal.
            # Logger.loginfo("Waiting image...")
            # if self.rgb_image is not None:
            #     Logger.loginfo("Get rgb image!")
            #     sys.stdout.flush()
            # else:
            #     Logger.logwarn(
            #         'Failed to get RGB image!')
            #     self._get_image_error = True
            # if self.depth_image is not None:
            #     Logger.loginfo("Get depth image!")
            #     sys.stdout.flush()
            # else:
            #     Logger.logwarn(
            #         'Failed to get Depth image!')
            #     self._get_image_error = True
            # if self.camera_info is not None:
            #     Logger.loginfo("Get camera info!")
            #     sys.stdout.flush()
            # else:
            #     Logger.logwarn(
            #         'Failed to get Camera info!')
            #     self._get_image_error = True

            self.start = timer()
            goal = hsr_common.msg.BoundingBox2DGoal()
            goal.image = self.bridge.cv2_to_imgmsg(self.rgb_image, "bgr8")

            # Send the goal.
            # make sure to reset the error state since a previous state execution might have failed
            try:
                self._client.send_goal(self._topic["img"], goal)
            except Exception as e:
                # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
                # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
                Logger.logwarn(
                    'Failed to send the BoundingBox2D command:\n%s' % str(e))
                self._error = True

            # Check if the client failed to send the goal.
            if self._get_image_error:
                return 'failed'
            elif self._error:
                return 'failed'

            # Check if the action has been finished
            rospy.sleep(0.2)
            if self._client.has_result(self._topic["img"]):
                # Logger.loginfo('Success for getting the result of YOLO detecttion.')
                result = self._client.get_result(self._topic["img"])
                self.end = timer()
                self.detect_sw = True
            else:
                self.detect_sw = False

            if self.detect_sw == True:

                # In this example, we also provide the amount of cleaned dishes as output key.
                # userdata.detection = result
                self.object_count = 0
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
                        pose.position.z = (cz * grasping_z/1000.0)+self.z_offset
                        pose.orientation.x = 0.0
                        pose.orientation.y = 0.0
                        pose.orientation.z = 0.0
                        pose.orientation.w = 0.0

                        try:
                            # transform and save the object poses
                            trans_pose = self.tf2_trans(pose)
                            self.pose_data = np.append(self.pose_data, [trans_pose.position.x,trans_pose.position.y,trans_pose.position.z])

                            # save the object labels
                            pre_object_data = [0 for c in xrange(self._data_dim["object"])]
                            pre_object_data[self.obj_class.index(result.names[n])] += 1
                            self.object_data = np.append(self.object_data, pre_object_data)

                            # count detection of object for word information
                            self.object_count += 1
                            Logger.loginfo("[Successfully]:Collect the information.")
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                            Logger.loginfo("[ERROR_tf2_time]:ignore the information. (obj:%s, id:%d)" %(result.names[n],len(self.pose_data)))
                            self.tf_buffer = tf2_ros.Buffer()
                            tf_listener = tf2_ros.TransformListener(self.tf_buffer)

                        cv2.rectangle(
                            self.rgb_image, (int(x0 + region.width*0.45 - 1), int(y0 + region.height*0.45 - 1)
                                             ), (int(x0 + region.width*0.55 - 1), int(y0 + region.height*0.55 - 1)),
                            (grasping_z/10000*255, grasping_z/10000 *
                             255, grasping_z/10000*255), cv2.FILLED
                        )
                #cv2.imshow("Image window", cv_image)
                # cv2.waitKey(3)

                # Get the place names
                pre_word_data = []
                if self.sub_word.has_msg(self._topic["wrd"]):
                    msg = self.sub_word.get_last_msg(self._topic["wrd"])
                    self.sub_word.remove_last_msg(self._topic["wrd"])
                    pre_word_data = [0 for i in xrange(self._data_dim["word"])]
                    pre_word_data[self.word_list.index(msg.data)] += 1
                    for m in xrange(self.object_count):
                        self.word_data = np.append(self.word_data, pre_word_data)
                else:
                    pre_word_data = np.zeros(self._data_dim["word"]*self.object_count)
                    self.word_data = np.append(self.word_data, pre_word_data)

                try:
                    self.image_pub.publish(
                        self.bridge.cv2_to_imgmsg(self.rgb_image, "bgr8")
                    )
                    if self._save_image:
                        img_name = self._save_folder+'/image/detect'+str(self.save_image_count)+'.png'
                        cv2.imwrite(img_name, self.rgb_image)
                        Logger.loginfo('Save the image: %s' %(img_name))
                        self.save_image_count += 1
                except cv_bridge.CvBridgeError as e:
                    Logger.logwarn(str(e))

                # Based on the result, decide which outcome to trigger.
                if not result is None:
                    Logger.loginfo('Finished detection. (%f [sec])' % (
                        self.end - self.start, ))
                else:
                    return 'failed'

                # If the action has not yet finished, no outcome will be returned and the state stays active.

        elif self.finish_sw == False and self._ignore == False:
            Logger.loginfo('Finished to get dataset.')
            re_pose = self.pose_data.reshape(len(self.pose_data)/self._data_dim["pose"],self._data_dim["pose"])
            re_object = self.object_data.reshape(len(self.object_data)/self._data_dim["object"],self._data_dim["object"])
            re_word = self.word_data.reshape(len(self.word_data)/self._data_dim["word"],self._data_dim["word"])

            np.savetxt(self._save_folder+"/pose.csv", re_pose)
            np.savetxt(self._save_folder+"/object.csv", re_object)
            np.savetxt(self._save_folder+"/word.csv", re_word)

            Logger.loginfo("Save the dataset to %s" %(self._save_folder))

            return 'completed'

        self._ignore = False

    def on_enter(self, userdata):
        # When entering this state, we send the action goal once to let the robot start its work.

        # As documented above, we get the specification of which dishwasher to use as input key.
        # This enables a previous state to make this decision during runtime and provide the ID as its own output key.
        self.image_pub = rospy.Publisher(
            self._output_topic, sensor_msgs.msg.Image, latch=True, queue_size=10
        )

        self._save_folder = self._save_folder+userdata.save_folder
        if os.path.exists(self._save_folder) == True:
            subprocess.Popen("rm -rf " + self._save_folder, shell=True)
        subprocess.Popen("mkdir -p " + self._save_folder + "/image", shell=True)


        self.pose_data = np.array([])
        self.word_data = np.array([])
        self.object_data = np.array([])

        self.word_list = np.loadtxt(self._save_folder+"/word_list.csv", dtype="unicode", delimiter="\n")
        self._data_dim.update({"word":len(self.word_list)})

        self.save_image_count = 0

        self.finish_sw = True
        self.detect_sw = False
        self._ignore = True

        self.tf_buffer = tf2_ros.Buffer()


    def on_exit(self, userdata):
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if not self._client.has_result(self._topic["img"]):
            self._client.cancel(self._topic["img"])
            Logger.loginfo('Cancelled active action goal.')

    def tf2_trans(self, before_pose):
        # tf_buffer = tf2_ros.Buffer()
        trans = self.tf_buffer.lookup_transform("map", "head_rgbd_sensor_rgb_frame", rospy.Time(0), rospy.Duration(1.0))#rospy.Time(0), rospy.Duration(1.0)) #rospy.Time.now()) rospy.get_rostime()
        pose = PoseStamped()

        pose.pose = before_pose
        transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)

        return transformed.pose

    def get_direction(self, camera_info, px, py):
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)
        cx, cy, cz = camera_model.projectPixelTo3dRay((px, py))
        return cx, cy, cz

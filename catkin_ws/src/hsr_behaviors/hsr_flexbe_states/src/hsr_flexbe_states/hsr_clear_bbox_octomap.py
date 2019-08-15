#!/usr/bin/env python
import rospy
from octomap_msgs.srv import BoundingBoxQuery
from octomap_msgs.srv import BoundingBoxQueryRequest
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Point

class hsr_ClearBBoxOctomap(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- topic_name          string

    -- bbox_param_x        float   Default: 0.2

    -- bbox_param_y        float   Default: 0.2

    ># object_point        Pose

    <= completed           clear the octomap

    <= failed              cannot clear the octomap

    '''

    def __init__(self, topic_name="octomap_server/clear_bbx", bbox_param_xy=0.2, bbox_param_z=0.4):
        super(hsr_ClearBBoxOctomap,self).__init__(outcomes=['completed','failed'],input_keys=['object_point'])
        self._topic = topic_name
        rospy.wait_for_service(self._topic)
        self.srv_clear_bbx = rospy.ServiceProxy(self._topic, BoundingBoxQuery)

        # cubes_of_candidates:
        # A list of 3D cubes.
        # Each cube corresponds to a 3D region including a detected object.
        # Each cube is represented as a BoundingBoxQueryRequest including two
        # keys 'min' and 'max' and their values are instances of Point.
        #
        # This state will remove obstacles in *ALL* cubes from the octmap
        # because which object will be grasped has not been decided in this
        # state.
        self.cubes_of_candidates = []

        self.param_xy = bbox_param_xy
        self.param_z = bbox_param_z

    def execute(self, userdata):
        try:
            for cube in self.cubes_of_candidates:
                # Call the service for clearing a bounding box.
                self.srv_clear_bbx(cube)
            return 'completed'
        except rospy.ServiceException, e:
            rospy.loginfo("[Service "+self._topic+"/client] call failed: %s", e)
            return 'failed'

    def on_enter(self, userdata):
        print('type(userdata.object_point): [%s]' %(type(userdata.object_point), ))

        if type(userdata.object_point) == type(dict()):
            object_points = userdata.object_point.values()
        else:
            object_points = [userdata.object_point]

        self.cubes_of_candidates = []
        for object_point in object_points:
            # Generate a cube corresponding to `object_point`.
            cube = BoundingBoxQueryRequest()
            cube.min.x = object_point.position.x - self.param_xy
            cube.min.y = object_point.position.y - self.param_xy
            cube.min.z = object_point.position.z - self.param_z
            cube.max.x = object_point.position.x + self.param_xy
            cube.max.y = object_point.position.y + self.param_xy
            cube.max.z = object_point.position.z + self.param_z
            self.cubes_of_candidates.append(cube)

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
import moveit_commander
import sys
import copy
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import hsrb_interface
from hsrb_interface import geometry
import copy

class hsr_CollisionBox(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.
    -- offset_z    float   Default: 0.3

    -- offset_dist    float   Default: 1.0

    -- width     float   Default: 0.7

    -- mode      string   Default: 'C'

    ># box_pose    pose    box_pose

    <= continue        continue
    '''

    def __init__(self, offset_z=0.3, offset_dist=0.1, width=0.7, mode='C'):
        super(hsr_CollisionBox,self).__init__(outcomes=['continue'],input_keys=['box_pose'])
        self.robot = hsrb_interface.Robot()
        self._box_frame_id = 'map'
        self.offset_z = offset_z
        self.offset_dist = offset_dist
        self.width = width
        self.mode = mode
        self.scene = moveit_commander.PlanningSceneInterface()

    def execute(self, userdata):
        return 'continue'

    def on_enter(self, userdata):
        box_name = "collision_box"
        if self.mode == 'C':
            whole_body = self.robot.get('omni_base')
            current_pose = whole_body.get_pose(ref_frame_id='map')
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.pose = copy.deepcopy(userdata.box_pose)
            dist = math.sqrt((current_pose.pos.x - box_pose.pose.position.x)**2 + (current_pose.pos.y - box_pose.pose.position.y)**2)

            box_pose.header.frame_id = self._box_frame_id
            box_pose.pose.position.z = (box_pose.pose.position.z - self.offset_z) / 2
            box_pose.pose.position.y = box_pose.pose.position.y + ((self.width / 2) - self.offset_dist) / dist * (box_pose.pose.position.y - current_pose.pos.y)
            box_pose.pose.position.x = box_pose.pose.position.x + ((self.width / 2) - self.offset_dist) / dist * (box_pose.pose.position.x - current_pose.pos.x)
            box_pose.pose.orientation = current_pose.ori

            self.scene.add_box(box_name, box_pose, size=(self.width, self.width, box_pose.pose.position.z*2))
        elif self.mode == 'R':
            self.scene.remove_world_object(box_name)

    def on_exit(self, userdata):
        self.current_pose = None
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

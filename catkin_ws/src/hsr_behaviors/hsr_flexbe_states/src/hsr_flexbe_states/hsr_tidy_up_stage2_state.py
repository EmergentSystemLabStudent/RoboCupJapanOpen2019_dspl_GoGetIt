#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from import_tidy_up_stage2 import *
import hsrb_interface

class hsr_TidyUpStage2(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    ># recognition      string  speech recognition
    ># grasp_object     string  grasping object name
    ># grasp_status     string  grasping object status

    #> move_pose        Pose    move goal pose
    #> xtion_point      Point   xtion goal pose
    #> arm_pose         Pose    arm goal pose
    #> object           string  unknown or known
    #> failed_objects   dict    object dict

    <= retry            speech recognition again
    <= grasp            grasp object
    <= put              put object
    <= kitchen          move to kitchen
    '''

    def __init__(self):
        super(hsr_TidyUpStage2,self).__init__(outcomes=['retry','grasp','put','kitchen'],input_keys=['recognition','grasp_object', 'grasp_status'],output_keys=['move_pose','xtion_point','arm_pose','object', 'failed_objects'])
        self.robot      = hsrb_interface.Robot()
        self.whole_body = self.robot.get('whole_body')

        self.text = str()
        self.object = str()
        self.object_status = str()
        self.place = str()
        self.next_state = "start"
        self.table_or_sofa = "table"
        self.unknown_or_known = "known"
        self.failed_objects = {}

        self.kitchen_unit_count = 0
        self.food_cabinet_count = 0
        self.coffee_table_count = 0
        self.wall_shelf_count = 0

        self.move_dict = {'table': table_move, 'sofa': sofa_move, 'kitchen':kitchen_move, 'kitchen unit':kitchen_unit_move, 'food cabinet': food_cabinet_move, 'coffee table': coffee_table_move, 'wall shelf': wall_shelf_move, 'trash': trash_move}
        self.arm_dict = {'kitchen unit': kitchen_unit_arm, 'food cabinet': food_cabinet_arm, 'coffee table': coffee_table_arm, 'wall shelf': wall_shelf_arm, 'trash': trash_arm}
        self.xtion_dict = {'table': table_xtion, 'sofa': sofa_xtion, 'kitchen unit': kitchen_unit_xtion, 'food cabinet': food_cabinet_xtion, 'coffee table': coffee_table_xtion, 'wall shelf': wall_shelf_xtion, 'trash': trash_xtion}


    def execute(self, userdata):
        if self.next_state == "retry":
            return 'retry'
        elif self.next_state == "grasp":
            userdata.object = self.unknown_or_known
            userdata.move_pose = self.move_dict.get(self.table_or_sofa)
            userdata.xtion_point = self.xtion_dict.get(self.table_or_sofa)
            userdata.failed_objects = self.failed_objects
            print "unknown_or_known"
            print self.unknown_or_known
            return 'grasp'
        elif self.next_state == "kitchen":
            userdata.move_pose = self.move_dict.get('kitchen')
            return 'kitchen'
        elif self.next_state == "put":
            userdata.move_pose = self.move_dict.get(self.place)
            userdata.xtion_point = self.xtion_dict.get(self.place)
            if self.place == "kitchen unit":
                userdata.arm_pose = self.arm_dict.get(self.place)[self.kitchen_unit_count]
                self.kitchen_unit_count += 1
            elif self.place == "food cabinet":
                userdata.arm_pose = self.arm_dict.get(self.place)[self.food_cabinet_count]
                self.food_cabinet_count += 1
            elif self.place == "coffee table":
                userdata.arm_pose = self.arm_dict.get(self.place)[self.coffee_table_count]
                self.coffee_table_count += 1
            elif self.place == "wall shelf":
                userdata.arm_pose = self.arm_dict.get(self.place)[self.wall_shelf_count]
                self.wall_shelf_count += 1
            elif self.place == "trash":
                userdata.arm_pose = self.arm_dict.get(self.place)
            return 'put'


    def on_enter(self, userdata):
        self.whole_body.move_to_go()

        self.text = userdata.recognition
        self.object = userdata.grasp_object
        self.object_status = userdata.grasp_status

        self.text = self.text.replace("-"," ")
        self.text = self.text.replace("  ", " ")

        if self.next_state == "start":
            self.next_state = "grasp"

        elif self.next_state == "kitchen":
            print('object_name:', self.object)
            print('object_status:', self.object_status)
            if self.object_status == "not_detect" or self.object_status == "not_grasp" or self.object_status == "fail_plan":
                self.next_state = "grasp"
            if self.object_status == "not_detect":
                self.unknown_or_known = "known"
            elif self.object_status == "not_grasp":
                self.unknown_or_known = "known"
                self.failed_objects.update({self.object:True})
            elif self.object_status == "fail_plan":
                self.unknown_or_known = "known"
                if self.object in self.failed_objects.keys():
                    self.failed_objects[self.object] = True
                else:
                    self.failed_objects.update({self.object:False})
            print("failed list: ", self.failed_objects)

        elif self.next_state == "put":
            if self.unknown_or_known == "unknown":
                if self.text == "Please discard it.":
                    self.place = "trash"
                elif "Please tidy up the " in self.text:
                    self.text = self.text.replace("Please tidy up the ","")
                    self.place = self.text.replace(".","")
                else:
                    self.next_state = "retry"
            elif self.unknown_or_known == "known":
                if self.object_status == "not_detect" or self.object_status == "not_grasp" or self.object_status == "fail_plan":
                    self.next_state = "grasp"
                else:
                    self.place = known_dict.get(self.object)
                if self.object_status == "not_detect":
                    self.unknown_or_known = "known"
                    if self.table_or_sofa == "table":
                        self.table_or_sofa = "sofa"
                        self.failed_objects = {}
                    elif self.table_or_sofa == "sofa":
                        self.table_or_sofa = "table"
                        self.failed_objects = {}
                elif self.object_status == "not_grasp":
                    self.unknown_or_known = "known"
                    self.failed_objects.update({self.object:True})
                elif self.object_status == "fail_plan":
                    self.unknown_or_known = "known"
                    if self.object in self.failed_objects.keys():
                        self.failed_objects[self.object] = True
                    else:
                        self.failed_objects.update({self.object:False})
                print("failed list: ", self.failed_objects)


    def on_exit(self, userdata):
        self.text = None
        self.object = None

        if self.next_state == "retry":
            self.next_state = "put"
        elif self.next_state == "grasp" and self.unknown_or_known == "unknown":
            self.next_state = "kitchen"
        elif self.next_state == "grasp" and self.unknown_or_known == "known":
            self.next_state = "put"
        elif self.next_state == "kitchen":
            self.next_state = "put"
        elif self.next_state == "put":
            self.next_state = "grasp"


    def on_start(self):
        pass

    def on_stop(self):
        pass

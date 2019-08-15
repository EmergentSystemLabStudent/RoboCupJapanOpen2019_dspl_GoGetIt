#!/usr/bin/env python
import rospy, os
import moveit_commander
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class ObjectCollision():
    def __init__(self):
        rospy.init_node('moveit_object_collision')

        #self.robot = moveit_commander.RobotCommander()
        #self.group = moveit_commander.MoveGroupCommander("whole_body")
        self.ps = moveit_commander.PlanningSceneInterface()
        self.model_name = None
        self.model_pose = None
        self.model_twist = None
        self.sw = True
        self.model_direc = "/root/HSR/catkin_ws/src/nrp_gazebo_worlds/models/"
        self.object_model_files = []
        self.match_id = []

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.sub_model_state, queue_size=10)
        rospy.Subscriber('/get_model_states', Bool, self.sub_sw, queue_size=1)

        '''
        while not rospy.is_shutdown():
            if self.sw != True:
                id = self.model_name.index("wrs_fcsc_emoticon_think")
                print "Add the model: " + self.model_name[id]
                #for m in xrange(len(self.model_name)):
                #    self.ps.add_mesh(self.model_name[m],self.model_pose_stamp,self.object_model_files[m],(1,1,1))
                self.model_pose_stamp.pose = self.model_pose[id]
                self.ps.add_mesh(self.model_name[id],self.model_pose_stamp,self.object_model_files[id],(1,1,1))# wrs_fcsc_game
            else:
                print("pass")
                pass
                r.sleep()
        '''

    def sub_sw(self,msg):
        self.sw = msg.data


    def sub_model_state(self,msg):
        if self.sw == True:
            self.model_name = msg.name
            self.model_pose = msg.pose
            self.model_pose_stamp = PoseStamped()
            self.model_pose_stamp.header.frame_id = "/map" #self.robot.get_planning_frame()
            #self.model_twist = msg.twist

            self.object_model_files = self.get_3dmodel_file_name()

            for m in self.match_id[1:]:
                print "Add the model: " + self.model_name[m]
                self.model_pose_stamp.pose = self.model_pose[m]
                self.ps.add_mesh(self.model_name[m],self.model_pose_stamp,self.object_model_files[m])#,(0.1,0.1,0.1))
                rospy.sleep(0.5)

            self.sw = False


    def get_3dmodel_file_name(self):
        object_model_files = []
        print "Getting Gazebo Models:"
        for f in xrange(len(self.model_name)):
            if self.model_name[f] in os.listdir(self.model_direc):
                object_model_files.append(self.model_direc+self.model_name[f]+"/meshes/"+self.model_name[f]+".dae")
                print self.model_name[f]
                self.match_id.append(f)
            else:
                object_model_files.append("None")
                print "Cannot Get 3D Model file: " + self.model_name[f]

        self.sw = False
        #if len(self.model_name) == len(object_model_files):
        return object_model_files
        #else:
        #    object_model_files = []
        #    return object_model_files


if __name__ == '__main__':
	Octo = ObjectCollision()
	rospy.spin()

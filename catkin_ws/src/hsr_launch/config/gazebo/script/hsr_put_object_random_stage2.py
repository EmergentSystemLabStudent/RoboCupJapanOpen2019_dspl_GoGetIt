#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, random, tf, os, sys
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool

class PutObjectsRandomly():
    def __init__(self):
        rospy.init_node('put_objects_randomly')
        r = rospy.Rate(0.01)
        # Setting
        self.put_places = ["hsr_white_table","hsr_dining_table","hsr_sofa_nocushion"] # "hsr_sofa" or "hsr_sofa_nocushion"
        exclusion_objects = ["hsr_keyboard"]
        self.urdf_direc = "/root/HSR/catkin_ws/src/hsr_launch/config/gazebo/urdf/"
        urdf_files = os.listdir(self.urdf_direc)
        pre_object_name = [f.replace(".urdf","") for f in urdf_files if os.path.isfile(os.path.join(self.urdf_direc, f))] # Get objects' name
        self.pre_object_name = [o for o in pre_object_name if o not in exclusion_objects] # Remove exclusion_objects
        self.object_name = [obj[4:] for obj in self.pre_object_name if "wrs_" or "nrp_" or "hsr_" or "3rd_"]

        self.SAMPLE_NUM = 5
        self.RANDOM_NUM = 100
        self.sw = True
        """# test
        self.MAP_MAX_X = RANDOM_NUM / 2.0 # max value map x (tc123: 3.0[m])
        self.MAP_MIN_X = RANDOM_NUM / 0.0 # min value map x (tc123: -2.5[m])
        self.MAP_MAX_Y = RANDOM_NUM / 1.0 # max value map y (tc123: 2.0[m])
        self.MAP_MIN_Y = RANDOM_NUM / 0.0  # min value map y (tc123: -0.5[m])
        """

        print("Waiting for gazebo services...")
        self.del_mdl = rospy.wait_for_service("/gazebo/delete_model")
        self.spw_mdl = rospy.wait_for_service("/gazebo/spawn_urdf_model")
        self.get_mdl = rospy.wait_for_service("/gazebo/get_model_state")

        self.sub_g_mdl = rospy.Subscriber('/gazebo/model_states', ModelStates, self.sub_model_state, queue_size=10)
        rospy.Subscriber('/run_put_objects_randomly', Bool, self.sub_sw, queue_size=1)

        r.sleep()

    def sub_sw(self, msg):
        self.sw = msg.data

    def sub_model_state(self, msg):
        if self.sw == True:
            print("Get Current Models:")
            self.cur_model = msg.name
            print([c for c in self.cur_model])
            self.run_main()

    def run_main(self):
        # try:
        print("Start\n")
        self.del_mdl = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spw_mdl = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
        self.get_mdl = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)

        # Delete Phase
        self.d_mdl = [m for m in self.cur_model if 'put' in m]
        if len(self.d_mdl) > 0:
            self.run_delete_model()

        # Spawn Phase
        self.run_spawn_model()

        # END
        print "Completed.\nIf you want to reset, please publish True by \"/run_put_objects_randomly\""
        self.sw = False
        #self.sub_g_mdl.unregister()
        #sys.exit(0)
        # except:
        #     print "Found soming error."
        #     sys.exit(1)

    def run_delete_model(self):
        print "Delete objects:"
        print([d for d in self.d_mdl])
        for d in self.d_mdl:
            self.del_mdl(d)

    def run_spawn_model(self):
        print("Get table and sofa Models:")
        print([m for m in self.put_places])
        p_place = {}
        for p in self.put_places:
            p_place.update({p:self.get_mdl(p, "world")})

        for i in range(self.SAMPLE_NUM): #range(len(object_name)) # Select a object in order
            num = [random.randint(1,self.RANDOM_NUM), random.randint(0,len(self.put_places)-1)]

            switch = num[0] % len(self.object_name) # Select a object randomly
            #switch = i # Select a object in order

            with open(self.urdf_direc+self.pre_object_name[switch]+".urdf", "r") as f:
                product_xml = f.read()
            item_name = "put" + str(i+1) + "_" + self.object_name[switch] # Set each object name

            # hsr_white_table x:1.10 y:0.55 z:0.37
            if self.put_places[num[1]] == "hsr_white_table":
                put_pos_x = p_place["hsr_white_table"].pose.position.x + random.uniform(-0.075,0.075) # length/2 - 0.2
                put_pos_y = p_place["hsr_white_table"].pose.position.y + random.uniform(-0.35,0.35) # length/2 - 0.2

            # hsr_dining_table x:0.78 y:1.20 z:0.71
            if self.put_places[num[1]] == "hsr_dining_table":
                put_pos_x = p_place["hsr_dining_table"].pose.position.x + random.uniform(-0.4,0.4) # length/2 - 0.2
                put_pos_y = p_place["hsr_dining_table"].pose.position.y + random.uniform(-0.19,0.19) # length/2 - 0.2

            # hsr_sofa x:0.86 y:1.96 z:0.76
            if self.put_places[num[1]] == "hsr_sofa_nocushion":
                put_pos_x = p_place["hsr_sofa_nocushion"].pose.position.x+0.14 + random.uniform(-0.215,0.215) # length/4 # plus 0.14 because object do not hit the backrest of sofa
                put_pos_y = p_place["hsr_sofa_nocushion"].pose.position.y + random.uniform(-0.78,0.78) # length/2 - 0.3

            """# test
            put_pos_x = num[0] / self.MAP_MAX_X
            put_pos_y = num[0] / self.MAP_MAX_Y
            """

            # About z
            put_pos_z = 1.0

            print("Spawning model: " + self.object_name[switch] + " to " + self.put_places[num[1]])
            orient = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
            item_pose = Pose(Point(x=put_pos_x, y=put_pos_y, z=put_pos_z), orient)
            self.spw_mdl(item_name, product_xml, "", item_pose, "world")

if __name__ == '__main__':
    # try:
    por = PutObjectsRandomly()
    rospy.spin()
    # except:
    #     rospy.loginfo("put_objects_randomly node terminated.")

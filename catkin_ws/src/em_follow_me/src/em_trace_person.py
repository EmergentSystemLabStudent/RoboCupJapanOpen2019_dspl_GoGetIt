#!/usr/bin/env python
# coding: UTF-8

import rospy,tf
import numpy as np
import time,math
from std_msgs.msg import Bool
from scipy.stats import multivariate_normal
from sklearn.ensemble import RandomForestClassifier
from sklearn.cross_validation import train_test_split

from sensor_msgs.msg import LaserScan,PointCloud
from geometry_msgs.msg import Point,PointStamped,TransformStamped

IMAGE_WIDTH = 640.0
CAMERA_ANGLE = 77.0

DATA_SIZE = 201
DATA_LEN = 8

class Sensor_Information(object):

    def clustring_laser_info_by_distance(self,laser_info):

        cluster = np.arange(1,self.laser_size+1)
        for i in xrange(self.laser_size-1):
            short_r = laser_info[i+1:min(i+self.CLUSTERING_ITERATION,self.laser_size)]
            short_d = self.laser_rad[i+1:min(i+self.CLUSTERING_ITERATION,self.laser_size)]
            d = np.sqrt(laser_info[i]**2 + short_r**2 - 2 * laser_info[i] * short_r * np.cos(self.laser_rad[i] - short_d))
            same_claster = cluster[i+1:min(i+self.CLUSTERING_ITERATION,self.laser_size)][d<=self.CLUSTER_MIN_DIST]
            for j in np.unique(same_claster): cluster[cluster==j] = cluster[i]

        num = 1
        for i in np.unique(cluster):
            if np.sum(cluster==i) > self.CLUSTER_MIN_SIZE:
                cluster[cluster==i] = num
                num += 1
            else:
                cluster[cluster==i] = 0

        return num, cluster

    def clustring_laser_info_by_distance_new(self,laser_info):
        cluster = np.arange(1,self.laser_size+1)
        dist_cos = laser_info * self.laser_rad_cos
        dist_sin = laser_info * self.laser_rad_sin
        for i in xrange(self.laser_size-1):
            index_end = min(i+self.CLUSTERING_ITERATION,self.laser_size)
            dist_cos0 = dist_cos[i]
            dist_sin0 = dist_sin[i]
            short_dist_cos = dist_cos[i + 1:index_end]
            short_dist_sin = dist_sin[i + 1:index_end]
            d_square = (dist_cos0 - short_dist_cos) ** 2 + (dist_sin0 - short_dist_sin) ** 2
            same_claster = cluster[i + 1:index_end][d_square<=self.CLUSTER_MIN_DIST_SQUARE]
            for j in np.unique(same_claster): cluster[cluster==j] = cluster[i]

        num = 1
        for i in np.unique(cluster):
            if np.sum(cluster==i) > self.CLUSTER_MIN_SIZE:
                cluster[cluster==i] = num
                num += 1
            else:
                cluster[cluster==i] = 0

        return num, cluster

    def random_forest_for_laser_info_category(self,cluster_num,laser_info,cluster,means,covs):

        output = np.empty((cluster_num-1,DATA_LEN-1))
        for i in xrange(1,cluster_num):
            output[i-1,0] = np.sum(cluster==i)
            output[i-1,1:3] = means[i-1]
            output[i-1,3:5] = covs[i-1]
            output[i-1,5] = np.min(laser_info[cluster==i], axis=0)
            output[i-1,6] = np.max(laser_info[cluster==i], axis=0)
        return self.clf.predict_proba(output)

    def InitParticleCallback(self, data):
        self.init_particle_flag = False

    def LaserCallback(self,data):

        now = time.time()

        if not self.init_laser_flag:
            self.laser_rad = np.arange(data.angle_min,data.angle_max,data.angle_increment)
            if len(self.laser_rad) != len(data.ranges):
                # In an edge case, the length of 'data.ranges' may differ
                # from that of 'self.laser_rad'.
                # This is due to computation error and how to deal interval
                # end.
                # For example, the length of 'np.arange(1, 3, 0.5)' is 4 and
                # that of 'np.arange(1, 3+1.0e-6, 0.5)' is 5.
                #
                # In such a case, 'self.laser_rad' should be fixed
                # because this script assumes that 'data.ranges' and
                # 'self.laser_rad' have the same length.
                #
                num = len(data.ranges)
                inc = (data.angle_max - data.angle_min) / (num - 1)
                epsilon = 1.0e-6
                self.laser_rad = np.arange(
                    data.angle_min, data.angle_max + epsilon, inc
                )
            #
            self.laser_rad_cos = np.cos(self.laser_rad)
            self.laser_rad_sin = np.sin(self.laser_rad)
            self.laser_center = len(data.ranges) / 2 + 1
            self.laser_size = len(data.ranges)
            self.init_laser_flag = True
            self.deg_list = np.empty(0)
            self.raw_list = np.empty(0)
        laser_raw = np.array(data.ranges)

        # clustering
        cluster_num,cluster = self.clustring_laser_info_by_distance_new(laser_raw)
        # new_cluster_num,new_cluster = self.clustring_laser_info_by_distance_new2(laser_raw)
        # assert(cluster_num == new_cluster_num)
        # assert(np.all(cluster == new_cluster))

        means = np.empty((cluster_num-1,2))
        covs = np.empty((cluster_num-1,2))
        for i in xrange(1,cluster_num):
            means[i-1] = np.mean(zip(self.laser_rad[cluster==i],laser_raw[cluster==i]),axis=0)
            covs[i-1] = np.var(zip(self.laser_rad[cluster==i],laser_raw[cluster==i]),axis=0)

        # random forest
        try:
            prediction = self.random_forest_for_laser_info_category(cluster_num,laser_raw,cluster,means,covs)
        except Exception as e:
            return

        # particle filter
        if not self.init_particle_flag:
            self.particle = np.empty((self.PARTICLE_NUM,3))
            self.particle[:,0] = np.zeros(self.PARTICLE_NUM)
            self.particle[:,1] = np.ones(self.PARTICLE_NUM) * 1.0
            self.particle[:,0] += (np.random.random(self.PARTICLE_NUM) * self.PARTICLE_RAD_PARAM * 2 - self.PARTICLE_RAD_PARAM)
            self.particle[:,1] += (np.random.random(self.PARTICLE_NUM) * self.PARTICLE_RAW_PARAM * 2 - self.PARTICLE_RAW_PARAM)
            self.particle[:,2] = np.ones(self.PARTICLE_NUM) / self.PARTICLE_NUM

            self.init_particle_flag = True
        else:
            self.particle[:,0] += (np.random.random(self.PARTICLE_NUM) * self.PARTICLE_RAD_PARAM * 2 - self.PARTICLE_RAD_PARAM)
            self.particle[:,1] += (np.random.random(self.PARTICLE_NUM) * self.PARTICLE_RAW_PARAM * 2 - self.PARTICLE_RAW_PARAM)

        pdf_lookup_table = np.empty((len(self.particle), cluster_num-1))
        for j in xrange(1, cluster_num):
            pdf_lookup_table[:, j-1] = multivariate_normal.pdf(self.particle[:,0:2], means[j-1], covs[j-1])
        if prediction[:,1].max() > self.HUMAN_PROBA_THR:
            prediction[prediction[:,1] <= self.HUMAN_PROBA_THR, 1] = 0.0
            pps = (pdf_lookup_table * prediction[:,1]).sum(axis=1)
        else:
            pps = pdf_lookup_table.sum(axis=1)
        self.particle[:,2] += pps
        self.particle[:,2] = self.particle[:,2] / np.sum(self.particle[:,2])

        index = np.random.choice(np.arange(self.PARTICLE_NUM), self.PARTICLE_NUM, p=self.particle[:,2])
        self.particle = self.particle[index]

        # smoothing
        self.deg_list = np.append(self.deg_list,np.sum(self.particle[:,0]*self.particle[:,2]) / np.sum(self.particle[:,2]))
        self.raw_list = np.append(self.raw_list,np.sum(self.particle[:,1]*self.particle[:,2]) / np.sum(self.particle[:,2]))
        if len(self.deg_list) > self.SMOOTHING_LEN:
            self.deg_list = np.delete(self.deg_list,0)
            self.raw_list = np.delete(self.raw_list,0)
        smoothing_list = np.arange(len(self.deg_list))
        deg = np.sum(self.deg_list*smoothing_list) / np.sum(smoothing_list)
        raw = np.sum(self.raw_list*smoothing_list) / np.sum(smoothing_list)

        br = tf.TransformBroadcaster()
        #br.sendTransform((np.cos(deg)*raw,np.sin(deg)*raw,0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "trace_target", "base_range_sensor_link")

        # rviz
        if not self.OUTPUT_RVIZ: return
        hoge = PointCloud()
        hoge.header = data.header
        for x in xrange(1,cluster_num):
            p = Point()
            p.x = np.cos(means[x-1,0]) * means[x-1,1]
            p.y = np.sin(means[x-1,0]) * means[x-1,1]
            p.z = 0
            hoge.points.append(p)
        self.pub_rviz_mean.publish(hoge)
        hoge = PointCloud()
        hoge.header = data.header
        for x in xrange(self.PARTICLE_NUM):
            p = Point()
            p.x = np.cos(self.particle[x][0]) * self.particle[x][1]
            p.y = np.sin(self.particle[x][0]) * self.particle[x][1]
            p.z = 0
            hoge.points.append(p)
        self.pub_rviz_particle.publish(hoge)
        hoge = PointStamped()
        hoge.header = data.header
        hoge.point.x = np.cos(deg) * raw
        hoge.point.y = np.sin(deg) * raw
        self.pub_rviz_point.publish(hoge)

        # print "time : " + str(time.time() - now)


    def __init__(self):

        SCAN_TOPIC_NAME = rospy.get_param("~scan_topic_name")
        # param for clustering
        self.CLUSTERING_ITERATION = rospy.get_param("~clustering_iteration")
        self.CLUSTER_MIN_SIZE = rospy.get_param("~cluster_min_size")
        self.CLUSTER_MIN_DIST = rospy.get_param("~cluster_min_dist")
        self.CLUSTER_MIN_DIST_SQUARE = self.CLUSTER_MIN_DIST ** 2
        # param for random forest
        self.HUMAN_PROBA_THR = rospy.get_param("~human_proba_threshold")
        # param for particle filter
        self.PARTICLE_NUM = rospy.get_param("~particle_num")
        self.PARTICLE_RAD_PARAM = rospy.get_param("~particle_rad_param")
        self.PARTICLE_RAW_PARAM = rospy.get_param("~particle_raw_param")
        # param for smoothing
        self.SMOOTHING_LEN = rospy.get_param("~smoothing_len")
        # debag
        self.OUTPUT_RVIZ = rospy.get_param("~output_rviz")

        rospy.Subscriber(SCAN_TOPIC_NAME, LaserScan, self.LaserCallback,queue_size=1)
        rospy.Subscriber("init_particle", Bool, self.InitParticleCallback, queue_size=1)

        self.pub_rviz_mean = rospy.Publisher("katsu_mean", PointCloud, queue_size=1)
        self.pub_rviz_particle = rospy.Publisher("katsu_particle", PointCloud, queue_size=1)
        self.pub_rviz_point = rospy.Publisher("katsu_point", PointStamped, queue_size=1)

        self.init_particle_flag = False
        self.init_laser_flag = False
        self.is_robot_move = False
        self.is_yolo_newdata = False

        # model for random forest
        self.input = np.zeros((2,DATA_LEN))
        for x in xrange(DATA_SIZE):
            hoge = np.loadtxt("dataset_all/output" + str(x+1) + ".csv", delimiter=",")
            self.input = np.append(self.input, hoge, axis=0)
        self.input = np.delete(self.input, [0,1], 0)
        train_X = np.delete(self.input, DATA_LEN-1, 1)
        train_y = self.input[:,DATA_LEN-1]
        self.clf = RandomForestClassifier(random_state=0)
        self.clf = self.clf.fit(train_X, train_y)


if __name__ == '__main__':

    rospy.init_node('person_track', anonymous=True)

    Sensor_Information()

    rospy.spin()

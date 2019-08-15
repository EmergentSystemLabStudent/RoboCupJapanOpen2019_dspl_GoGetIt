#!/usr/bin/env python
# -*- coding: utf-8 -*-

# made by Tomohiro Mimura
# edit by Yuki Katsumata

from __init__ import *

from std_msgs.msg import Int32
import scipy.stats as ss
import random
import sklearn.cluster
import sklearn.metrics
import time
import subprocess

class SpCoF():

    def __init__(self,pose,word_feature,image_feature,num_class):
        self.num_iter  = ITERATION
        self.num_class = num_class

        self.pose      = pose
        self.word_feature     = word_feature
        self.image_feature     = image_feature

        self.gamma    = np.ones([num_class])*GAMMA
        self.pi        = self.stick_breaking(GAMMA,self.num_class)
        self.alpha1    = np.ones([word_feature.shape[1]])*BETA
        self.alpha2    = np.ones([image_feature.shape[1]])*CHI
        self.phai1     = ss.dirichlet.rvs(self.alpha1,num_class)
        self.phai2     = ss.dirichlet.rvs(self.alpha2,num_class)
        self.C_t       = np.random.multinomial(1,self.pi,size=word_feature.shape[0])

        self.V         = np.eye(pose.shape[1])*ALPHA #np.cov(data,rowvar=0) /(10)
        self.v0        = V0 #pose.shape[1]+1
        self.m0        = np.mean(pose, axis=0)
        self.k0        = K0

        self.mu        =  sklearn.cluster.KMeans(n_clusters=num_class, random_state=random.randint(1,100)).fit(pose).cluster_centers_
        self.sigma     =  [self.V for n in xrange(self.num_class)]

        self.pub_learned = rospy.Publisher("/em/spco_formation/learned", Int32, queue_size = 1)


    def stick_breaking(self,gamma,num_class):
        betas = np.random.beta(1, gamma, num_class)
        remaining_pieces = np.append(1, np.cumprod(1 - betas[:-1]))
        p = betas * remaining_pieces
        return p/p.sum()

    def fit(self):
        for gibbs in range(self.num_iter):
            self.gibbs_sampling()

        Ct = np.sum(self.C_t, axis=0)
        self.spco_num = np.sum(Ct > 0)
        self.pub_learned.publish(self.spco_num)
        rospy.loginfo("[Service spcof/learn] SpCo num : %d", self.spco_num)

        RESULT_FOLDER = RESULT_PATH + TRIALNAME
        np.savetxt(RESULT_FOLDER + "/mu.csv", self.mu)
        np.savetxt(RESULT_FOLDER + "/pi.csv", self.pi)
        np.savetxt(RESULT_FOLDER + "/word.csv", self.phai1)
        np.savetxt(RESULT_FOLDER + "/Ct.csv", Ct)
        for x in xrange(self.num_class):
            np.savetxt(RESULT_FOLDER + "/sigma/" + str(x) + ".csv", self.sigma[x])

    def gibbs_sampling(self):
        self.phai1   =  self.dirichlet_multinomial1(self.alpha1, self.word_feature,self.C_t)
        self.phai2   =  self.dirichlet_multinomial1(self.alpha2, self.image_feature,self.C_t)
        self.pi      =  self.dirichlet_multinomial2(self.gamma, self.C_t)
        self.C_t     =  self.multinomial_multinomial_gaussian()
        self.sigma, self.mu = self.gaussian_inverse_wishart(self.pose)

    def dirichlet_multinomial1(self,eta,data,z_i):
        beta = np.ones([self.num_class,data.shape[1]])
        for i in range(self.num_class):
            beta[i]  = ss.dirichlet.rvs((z_i[:,i]*data.T).sum(1)+eta)
        return beta

    def dirichlet_multinomial2(self,alpha,z_i):
        return ss.dirichlet.rvs(alpha+z_i.sum(0))

    def multinomial_multinomial_gaussian(self):
        log_phai1  = np.log(self.phai1)
        log_phai2  = np.log(self.phai2)
        z_i        = np.zeros([self.word_feature.shape[0],self.num_class])
        log_likely = self.word_feature.dot(log_phai1.T)+self.image_feature.dot(log_phai2.T)
        sub_pi     = self.pi * np.exp(log_likely-np.array([log_likely.max(1) for j in range(self.num_class)]).T)
        probab     = sub_pi/np.array([sub_pi.sum(1) for j in range(self.num_class)]).T

        self.new_pi = np.zeros((self.pose.shape[0],self.num_class))
        for i in xrange(self.num_class):
            self.new_pi[:,i] = ss.multivariate_normal.pdf(self.pose,self.mu[i],self.sigma[i])*self.pi[0,i]

        self.new_pi = self.new_pi * probab
        for i in range(self.word_feature.shape[0]):
            if self.new_pi.sum(1)[i] != 0:
                self.pi_sub = (self.new_pi[i].T/self.new_pi.sum(1)[i]).T
                z_i[i]      = np.random.multinomial(1,self.pi_sub,size=1)
            else:
                self.pi_sub = np.ones(self.word_feature.shape[0]) / self.word_feature.shape[0]
                z_i[i]      = np.random.multinomial(1,self.pi_sub,size=1)
        return z_i

    def gaussian_inverse_wishart(self,pose):
        hist           = self.C_t.sum(0)
        self.mean      = np.zeros((self.num_class,pose.shape[1]))

        self.Vn        = [np.linalg.inv(self.V) for i in xrange(self.num_class)]
        self.mn        = [self.m0 * self.k0     for i in xrange(self.num_class)]
        self.kn        = self.k0  + self.C_t.sum(0)
        self.vn        = self.v0  + self.C_t.sum(0)
        for i in xrange(self.num_class):
            if hist[i] != 0:
                self.sub_pose   =  pose * np.array([self.C_t[:,i] for j in xrange(pose.shape[1])]).T
                self.mean[i]    =  self.sub_pose.sum(0)/float(hist[i])
                self.mn[i]      = (self.mn[i]   + self.sub_pose.sum(0))/self.kn[i]
                self.var        =  np.array([self.mean[i] for t in range(pose.shape[0])]) * np.array([self.C_t[:,i] for j in xrange(pose.shape[1])]).T
                self.Vn[i]     += (self.sub_pose - self.var).T.dot(self.sub_pose - self.var) + \
                                    self.k0 * hist[i] / self.kn[i] * (self.mean[i] -self.m0)[:, np.newaxis].dot((self.mean[i] -self.m0)[np.newaxis,:])

                self.sigma[i]   =  ss.invwishart.rvs(self.vn[i],self.Vn[i])
                self.mu[i]      =  ss.multivariate_normal.rvs(self.mn[i], np.linalg.inv(self.Vn[i]) / self.kn[i])

            else:
                self.sigma[i]   =  ss.invwishart.rvs(self.v0,self.V)
                self.mu[i]      =  ss.multivariate_normal.rvs(self.m0, np.linalg.inv(self.V) / self.k0)

        return self.sigma,self.mu


def run_server(req):

    RESULT_FOLDER = RESULT_PATH + TRIALNAME
    p = subprocess.Popen("rm -rf " + RESULT_FOLDER, shell=True)
    rospy.sleep(1.0)
    p = subprocess.Popen("mkdir -p " + RESULT_FOLDER + "/sigma/", shell=True)

    try:
        DATA_FOLDER = DATASET_FOLDER + TRIALNAME
        pose      = np.loadtxt(DATA_FOLDER + "/pose.csv"    ,delimiter=",")
        word      = np.loadtxt(DATA_FOLDER + "/word.csv"    ,delimiter=",")
        image     = np.loadtxt(DATA_FOLDER + "/image.csv"   ,delimiter=",")
    except Exception as e:
        rospy.loginfo("[Service spcof/learn] %s", e)
        return spcof_learnResponse(False, 0)

    rospy.loginfo("[Service spcof/learn] start")
    start     = time.time()

    s     = SpCoF(pose,word,image,req.data_num)
    s.fit()

    elapsed_time = time.time() - start
    rospy.loginfo("[Service spcof/learn] time : %f", elapsed_time)

    return spcof_learnResponse(True, s.spco_num)


if __name__ == "__main__":

    rospy.init_node('spcof_learn_server')
    TRIALNAME = rospy.get_param('~trial_name')

    s = rospy.Service('em_spco_formation/learn', spcof_learn, run_server)
    rospy.loginfo("[Service spcof/learn] Ready em_spco_formation/learn")

    rospy.spin()

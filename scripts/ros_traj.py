#!/usr/bin/python
# -*- coding: utf-8 -*-

""" Alexandre Coninx
    ISIR CNRS/UPMC
    08/06/2017
""" 

import numpy as np

import rospy

import balltraj

from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import PoseStamped

import matplotlib
#matplotlib.use("GTKAgg")

import filternew_secondorder

trajectory_data_topic = "/ball_trajectory"
trajectory_data_msgtype = Float64MultiArray

ts_data_topic = "/trajectory_time_stamps"
ts_data_msgtype = Float64MultiArray

gripper_data_topic = "/gripping_status"
gripper_data_msgtype = Float64MultiArray

intersect_data_topic = "/baxter_throwing/contact_pose"
intersect_data_msgtype = PoseStamped




basket_pos_topic = "/basket_position" #"/baxter_throwing/basket_pose"
basket_pos_data_msgtype = Float64MultiArray # PoseStamped


class RosTrajectoryAnalysis:
	def __init__(self, mode='kalman'):
		rospy.init_node('ros_traj_analysis', anonymous=True)
		self.rate = rospy.Rate(10) #10Hz should be enough
		self.mode=mode
		# Data from Ghanim
		self.trajdata_sub = rospy.Subscriber(trajectory_data_topic, trajectory_data_msgtype, self.cb_trajdata)
		self.trajdata_sub = rospy.Subscriber(ts_data_topic, ts_data_msgtype, self.cb_tsdata)
		self.trajdata_sub = rospy.Subscriber(gripper_data_topic, gripper_data_msgtype, self.cb_gripperdata)
		self.basketsub = rospy.Subscriber(basket_pos_topic,basket_pos_data_msgtype, self.cb_basket)
		# Out
		self.intersect_pub = rospy.Publisher(intersect_data_topic, intersect_data_msgtype, queue_size=1)
		# Internal
		self.filter = filternew_secondorder.KFballtrackOutliers()
		# Inits
		self.basket_position = balltraj.basket_top_location
		out = balltraj.load_traj_with_ts_and_state("throw_ball_trials_2/10_th/ball_trajectory_robot_frame_2.csv")		
		self.current_trajdata = out[0]
		self.current_tsdata = out[1]
		self.current_gripperdata = out[1]
		# Output data
		self.intersect_pos = None
		self.intersect_ts = None
		self.seq=0
		self.n_basket=0
		self.n_traj=0
		self.n_ts=0
		self.n_gripper=0
		
	def cb_trajdata(self,msg):
                print("Received ball trajectory data and processing")
		rawdata = np.array(msg.data)
		npoints = len(rawdata)/3
		self.current_trajdata = np.reshape(rawdata,(npoints,3))
		self.n_traj +=1
	
	def cb_tsdata(self,msg):
                print("Received time stamp data and processing")
		rawdata = np.array(msg.data)
		self.current_tsdata = rawdata
		self.n_ts +=1
	
	def cb_gripperdata(self,msg):
                print("Received gripping data and processing")
		rawdata = np.array(msg.data,dtype=int)
		self.current_gripperdata = rawdata
		self.n_gripper +=1

	def cb_basket(self,msg):
                print("Received basket data and processing")
		rawdata = np.array(msg.data)
		npoints = len(rawdata)/3
		self.basket_position = np.reshape(rawdata,(npoints,3)).mean(axis=0) # Should be robust even if Ghanim directly publishes the mean
		self.n_basket +=1


	def check_process(self):
		if(self.n_basket == (self.seq+1) and self.n_gripper == (self.seq+1) and self.n_ts == (self.seq+1) and self.n_traj == (self.seq+1)):
			print("Processing and publishing new trajectory (#%d)" % self.seq)
			self.process_tdata()
			self.publish_intersect()
		

	def process_tdata(self):
                print ("ball trajectory is: ")
                print (self.current_trajdata)
                print ("basket position is: ")
                print (self.basket_position)
                print ("gripper data is: ")
                print (self.current_gripperdata)
                print ("time stamp data is: ")
                print (self.current_tsdata)

		self.filter.reset_filter(initpos=self.current_trajdata[0,:])
		out_x, out_sigma2, out_pol, index_throw = filternew_secondorder.test_filter_outlier_dt_state(self.filter,(self.current_trajdata, self.current_tsdata, self.current_gripperdata))
		intersect_tr, success_tr, time_tr = balltraj.locate_basket_intersection(self.current_trajdata,index_throw=index_throw,basket_xy_coords=self.basket_position[:2],basket_level=self.basket_position[2],ts=self.current_tsdata)
		intersect_kf, success_kf, time_kf = balltraj.locate_basket_intersection(out_x,index_throw=index_throw,basket_xy_coords=self.basket_position[:2],basket_level=self.basket_position[2],ts=self.current_tsdata)
		print("Intersection according to the raw trajectory : %s" % str(intersect_tr))
		print("Intersection according to the Kalman filter : %s" % str(intersect_kf))
		if(self.mode=='kalman'):
			self.intersect_pos = intersect_kf
			self.intersect_ts = time_kf
		else:
			self.intersect_pos = intersect_tr
			self.intersect_ts = time_tr
		self.seq += 1
		#return ((intersect_tr, success_tr, time_tr),(intersect_kf, success_kf, time_kf),out_x)
	
	def publish_intersect(self):
		if(self.intersect_pos is not None):
			msg = intersect_data_msgtype()
			msg.header.seq = self.seq
			sec = int(np.fix(self.intersect_ts))
			nsec = int(np.round((self.intersect_ts - sec)*1e9))
			#msg.header.stamp.sec = sec
			#msg.header.stamp.nsec = nsec
			msg.pose.position.x = self.intersect_pos[0]
			msg.pose.position.y = self.intersect_pos[1]
			msg.pose.position.z = self.intersect_pos[2]
			self.intersect_pub.publish(msg)
			
			
	def start_node(self):
		while not rospy.is_shutdown():
			self.check_process()
			self.rate.sleep()
			
			
if(__name__ == '__main__'):
	rta = RosTrajectoryAnalysis(mode="kalman") # mode="raw" to use raw trajectory data
	rta.start_node()

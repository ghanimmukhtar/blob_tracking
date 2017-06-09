#!/usr/bin/python
# -*- coding: utf-8 -*-

""" Alexandre Coninx
    ISIR CNRS/UPMC
    07/06/2017
""" 

import numpy as np

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag
from filterpy.stats import plot_covariance_ellipse
from matplotlib import cm

import matplotlib.pyplot as plt

import balltraj




init_pos_ball_default = np.array([0.8,-1.75,-0.7])

z_threshold  = balltraj.basket_top_location[2]




class KFballtrack:
	def __init__(self,initpos=init_pos_ball_default):
		self.kf = KalmanFilter(dim_x=6,dim_z=3)
		
		dt = 1.   # time step 1 second
		#dt = 0.07
		
		# State trans
		F_base = np.array([[1, dt],
		                      [0,  1]])
		self.kf.F = block_diag(F_base,F_base,F_base)
							  
		# State noise
		q = Q_discrete_white_noise(dim=2, dt=dt, var=0.001)
		self.kf.Q = block_diag(q, q, q)
		
		# Obs
		self.kf.H = np.array([[1.,0,0,0,0,0],[0,0,1.,0,0,0],[0,0,0,0,1.,0]])
		
		# Measure noise
		sigma_measure = 0.1**2
		
		self.kf.R = np.eye(3)*sigma_measure
		
		# Initial conditions : position
		self.kf.x = np.array([initpos[0],0.,initpos[1],0,initpos[2],0])
		init_cov = np.diag([0.1,10]) # Base value is reasonably well known, base velocity is not
		self.kf.P = block_diag(init_cov,init_cov,init_cov)
	
	def add_data(self,z):
		self.kf.predict()
		self.kf.update(z)
		y = self.kf.y
		log_likelihood = self.kf.log_likelihood
		return (y, log_likelihood) #y=residuals
		
	def get_out(self):
		return np.array(self.kf.x[::2])
	
	def get_sigma2(self):
		return np.array(self.kf.P.diagonal()[::2])



class KFballtrackOutliers:
	def __init__(self,initpos=init_pos_ball_default):
		self.kf_ok = KalmanFilter(dim_x=9,dim_z=3)
		self.kf_outlier = KalmanFilter(dim_x=9,dim_z=3)
		
		self.dt = 0.08   # time step
		#self.dt = 1.
			
		self.gripper_closed = True
		
		#Initialize transition matrices
		self.update_dt_and_state(self.dt,self.gripper_closed)
		
		# Obs
		h = np.zeros((3,9))
		h[0,0] = 1.0
		h[1,3] = 1.0
		h[2,6] = 1.0
		#h = np.array([[1.,0,0,0,0,0],[0,0,1.,0,0,0],[0,0,0,0,1.,0]])
		self.kf_ok.H = h
		self.kf_outlier.H = h
		
		# Measure noise
		sigma_measure = 0.1**2
		sigma_measure_outlier = 1.5**2
		
		self.kf_ok.R = np.eye(3)*sigma_measure
		self.kf_outlier.R = np.eye(3)*sigma_measure_outlier
		
		self.reset_filter(initpos=initpos)
		
		self.p_outlier = 0.01 # Initial probability of being an outlier
		self.p_ot1_otm0 = 0.1 # Probability of having an outlier after a regular point
		self.p_ot1_otm1 = 0.2 # Probability of having an outlier after another outlier


	def reset_filter(self,initpos=init_pos_ball_default):
		# Initial conditions : position
		self.kf_ok.x = np.array([initpos[0],0.,0.,initpos[1],0,0,initpos[2],0,0])
		self.kf_outlier.x = np.array([initpos[0],0.,0,initpos[1],0,0,initpos[2],0,0])
		init_cov = np.diag([0.01,0.1,0.1]) # Base value is reasonably well known, base velocity is unknown
		self.kf_ok.P = block_diag(init_cov,init_cov,init_cov)
		self.kf_outlier.P = block_diag(init_cov,init_cov,init_cov)
		self.gripper_closed = True

	def update_dt_and_state(self,dt,gripper_closed,justopened=False):#,just_closed,passed_thres):
		self.dt = dt
		self.gripper_closed = gripper_closed
		# Transition function
		

		
		# State noise
		
		
		if(gripper_closed):
			# While the ball is being manipulated by the robot, the acceleration is driven
			# by the robot and is only weakly related from one moment to the other
			F_base = np.array([[1, dt,0],[0,  1, dt],[0,0,0.01]])
			# We don't know what the robot is doing, so model it with a high state variance
			q_var = 0.5
		else:
			# Acceleration remains constantish
			F_base = np.array([[1, dt,0],[0,  1, dt],[0,0,1.]])
			# After release, the ball follows Newtonian dynamics, so our model is much better
			q_var = 0.05
			if(justopened): # We don't know well what happens at release time, add tons of variance and no correlation here
				F_base = np.array([[1, dt,0],[0,  1, dt],[0,0,0.01]])
				q_var = 100
		
		# Outliers have no physical model, assume low correlation
		F_base_outliers = np.array([[1, dt,0],[0,  0.1, dt],[0,0,0.1]])
		# ...and huge variance
		q_var_outliers = 50
		
		self.kf_ok.F = block_diag(F_base,F_base,F_base)
		self.kf_outlier.F = block_diag(F_base_outliers,F_base_outliers,F_base_outliers)
		q = Q_discrete_white_noise(dim=3, dt=dt, var=q_var)
		q_outlier =  Q_discrete_white_noise(dim=3, dt=dt, var=q_var_outliers)
		self.kf_ok.Q = block_diag(q, q, q)
		self.kf_outlier.Q = block_diag(q_outlier, q_outlier, q_outlier)

	
	def add_data(self,z):
		self.kf_ok.predict()
		self.kf_outlier.predict()
		self.kf_ok.update(z)
		self.kf_outlier.update(z)
		#y = self.kf.y
		log_likelihood_ok = self.kf_ok.log_likelihood
		log_likelihood_outlier = self.kf_outlier.log_likelihood
		
		#Compute probability of the obs being an outlier
		lhood_ok = np.exp(log_likelihood_ok)*(self.p_outlier*self.p_ot1_otm1 +(1.-self.p_outlier)*self.p_ot1_otm0)
		lhood_outlier = np.exp(log_likelihood_outlier)*(self.p_outlier*(1.-self.p_ot1_otm1) +(1.-self.p_outlier)*(1.-self.p_ot1_otm0))
		self.p_outlier = lhood_outlier/(lhood_ok+lhood_outlier)
		
		#Update data with weighted average for both filters
		P = self.p_outlier*self.kf_outlier.P + (1.-self.p_outlier)*self.kf_ok.P
		x = self.p_outlier*self.kf_outlier.x + (1.-self.p_outlier)*self.kf_ok.x
		self.kf_ok.P = P
		self.kf_outlier.P = P
		self.kf_ok.x = x
		self.kf_outlier.x = x
		
		return (log_likelihood_ok,log_likelihood_outlier,	self.p_outlier)	 #y=residuals

	def add_data_dt_state(self,z,dt,state,justopened=False):
		self.update_dt_and_state(dt,state,justopened=False)
		return self.add_data(z)

		
	def get_out(self):
		return np.array(self.kf_ok.x[::3])
	
	def get_sigma2(self):
		return np.array(self.kf_ok.P.diagonal()[::3])


def test_filter_outlier(f,traj):
	out_x = np.zeros_like(traj)
	out_sigma2 = np.zeros_like(traj)
	out_pol = np.zeros_like(traj)
	for i in range(traj.shape[0]):
		llok, llout, pol = f.add_data(traj[i,:])
		out_pol[i,:] = pol
		out_x[i,:] = f.get_out()
		out_sigma2[i,:] = f.get_sigma2()
	return (out_x, out_sigma2, out_pol)
	


def test_filter_outlier_dt_state(f,outdata):
	traj, ts, state = outdata
	out_x = np.zeros_like(traj)
	out_sigma2 = np.zeros_like(traj)
	out_pol = np.zeros_like(traj)
	dt = f.dt
	index_throw = 0
	for i in range(traj.shape[0]):
		justopened = False
		if(i>0):
			dt = ts[i]-ts[i-1]
			if((state[i] == 0) and (state[i-1] == 1)):
				justopened = True
				index_throw = i
		#dt=1.0
		llok, llout, pol = f.add_data_dt_state(traj[i,:],dt,state[i],justopened)
		out_pol[i,:] = pol
		out_x[i,:] = f.get_out()
		out_sigma2[i,:] = f.get_sigma2()
	return (out_x, out_sigma2, out_pol, index_throw)
	



def process_and_plot_traj_stateandt(tfile,kf=None):
	data = balltraj.load_traj_with_ts_and_state(tfile)
	initpos = data[0][0,:]
	if(kf != None):
		kfo = kf
		kfo.reset_filter(initpos)
	else:
		kfo = KFballtrackOutliers(initpos=initpos)
	out_x, out_sigma2, out_pol, index_throw = test_filter_outlier_dt_state(kfo,data)
	fig = plot_output(data[0],out_x,out_sigma2,out_pol,data[2],data[1])
	plt.suptitle("Trajectory cleaning - %s" % tfile)
	return out_x, out_sigma2, out_pol, index_throw





def plot_output(traj,pred,sigma2,pol,statevector=None, xdata=None):
	f, subaxes = plt.subplots(1, 3, sharey=True)
	cmap = cm.rainbow
	x_range = xdata if xdata != None else np.arange(traj.shape[0])
	axes = ["x","y","z"]
	if(statevector != None):
		releasemoment = int(np.argwhere(statevector==0)[0])
	else:
		releasemoment = traj.shape[0]
	for (i,ax) in enumerate(subaxes):
		ax.set_title("Axis %s" % axes[i])
		ax.plot(x_range[:releasemoment],traj[:releasemoment,i],color='black',label="Observation")
		if(releasemoment<traj.shape[0]):
			ax.plot(x_range[releasemoment:],traj[releasemoment:,i],color='blue')
		scplot = ax.scatter(x_range,traj[:,i],marker='o',cmap=cmap,c=pol[:,i],vmin=0.,vmax=1.)
		ax.plot(x_range,pred[:,i],color='green',label="Filter prediction")
		ax.fill(np.concatenate([x_range, x_range[::-1]]), np.concatenate([pred[:,i] - 1.9600 * sigma2[:,i],(pred[:,i] + 1.9600 * sigma2[:,i])[::-1]]),alpha=.3, fc='b', ec='None', label='95% conf. int.')
		if(i==0):
			ax.legend()
			bar = f.colorbar(scplot,ticks=[0., 0.2,0.4,0.6,0.8, 1]);
			bar.ax.set_ylabel("Outlier probability")
		if(i==2):
			ax.plot(x_range,[z_threshold]*len(x_range),color='orange')
	return f









def process_and_plot_traj(tfile):
	t = balltraj.load_traj(tfile)
	kfo = KFballtrackOutliers()
	out_x, out_sigma2, out_pol = test_filter_outlier(kfo,t)
	fig = plot_output(t,out_x,out_sigma2,out_pol)
	plt.suptitle("Trajectory cleaning - %s" % tfile)

	
def test_filter(f,traj):
	out_x = np.zeros_like(traj)
	out_sigma2 = np.zeros_like(traj)
	out_residuals = np.zeros_like(traj)
	out_ll = np.zeros_like(traj)
	for i in range(traj.shape[0]):
		residuals, ll = f.add_data(traj[i,:])
		out_residuals[i,:] = residuals
		out_ll[i,:] = ll
		out_x[i,:] = f.get_out()
		out_sigma2[i,:] = f.get_sigma2()
	return (out_x, out_sigma2, out_residuals, out_ll)
	
#!/usr/bin/python
# -*- coding: utf-8 -*-

""" Alexandre Coninx
    ISIR CNRS/UPMC
    07/06/2017
""" 

import numpy as np
import csv

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d



basket_top_location = np.array([1.85,0.,-0.55])
basket_radius = 0.14
basket_height = 0.50


def plot_3D_cylinder(ax,radius, height, elevation=0, resolution=100, color='r', x_center = 0, y_center = 0,alpha=0.2):
    #fig=plt.figure()
    #ax = Axes3D(fig, azim=30, elev=30)

    x = np.linspace(x_center-radius, x_center+radius, resolution)
    z = np.linspace(elevation, elevation+height, resolution)
    X, Z = np.meshgrid(x, z)

    Y = np.sqrt(radius**2 - (X - x_center)**2) + y_center # Pythagorean theorem

    ax.plot_surface(X, Y, Z, linewidth=0, color=color,alpha=alpha)
    ax.plot_surface(X, (2*y_center-Y), Z, linewidth=0, color=color,alpha=alpha)

    floor = Circle((x_center, y_center), radius, color=color,alpha=alpha)
    ax.add_patch(floor)
    art3d.pathpatch_2d_to_3d(floor, z=elevation, zdir="z")

    ceiling = Circle((x_center, y_center), radius, color=color,alpha=alpha)
    ax.add_patch(ceiling)
    art3d.pathpatch_2d_to_3d(ceiling, z=elevation+height, zdir="z")

    ax.set_xlabel('x-axis')
    ax.set_ylabel('y-axis')
    ax.set_zlabel('z-axis')

    plt.show()



def plot_basket(ax):
	plot_3D_cylinder(ax,radius=basket_radius,height=basket_height,elevation=(basket_top_location[2]-basket_height),color='b',x_center=basket_top_location[0],y_center=basket_top_location[1],resolution=10,alpha=0.3)

def load_traj(fname):
	out = list()
	with open(fname,'r') as fd:
		reader = csv.reader(fd,delimiter=',')
		for line in reader:
			p = np.array([float(v) for v in line])
			out.append(p)
	return np.array(out)[:,:3]

def load_traj_with_ts_and_state(fname):
	out = list()
	with open(fname,'r') as fd:
		reader = csv.reader(fd,delimiter=',')
		for line in reader:
			p = np.array([float(v) for v in line])
			out.append(p)
	out = np.array(out)
	pos = out[:,:3]
	ts = out[:,3]
	state = np.array(out[:,4],dtype=int)
	return pos, ts, state




def plot_traj(t):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.set_xlim(-0.5,2.5)
	ax.set_ylim(-1.5,1.5)
	ax.set_zlim(-1.5,2.5)
	#ax.set_aspect('equal')
	plot_basket(ax)
	
	for i in range(t.shape[0]-1):
		ax.plot((t[i,0],t[i+1,0]),(t[i,1],t[i+1,1]),(t[i,2],t[i+1,2]),color='black',marker='+')#red',lw=3)
		#ax.plot((t[i,0],t[i+1,0]),(t[i,2],t[i+1,2]),(t[i,1],t[i+1,1]),color='black',marker='+')#red',lw=3)
	


def locate_basket_intersection(traj,index_throw=1,basket_xy_coords=basket_top_location[:2], basket_level=basket_top_location[2], basket_radius=basket_radius,ts=None):
	gotbelow = (traj[-1,2] < basket_level)
	if(not gotbelow):
		print("WARNING trajecty finished above basket level")
	
	for i in range(index_throw-1,traj.shape[0])[::-1]:
		below =  traj[i,2] < basket_level
		if(gotbelow and not below):
			break
		elif below:
			gotbelow = True
	interpolation_coord = np.abs((basket_level - traj[i][2])/(traj[i+1][2] - traj[i][2]))
	intersect = interpolation_coord*traj[i+1,:] + (1 - interpolation_coord)*traj[i,:]
	success = np.sqrt(np.sum((intersect[:2] - basket_xy_coords)**2)) < basket_radius
	if(ts is not None):
		time = interpolation_coord*ts[i+1] + (1 - interpolation_coord)*ts[i]
	else:
		time = i
	return intersect, success, time
		

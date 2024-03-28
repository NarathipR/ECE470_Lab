#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
L2=0.120
L3=0.224
L4=0.093
L5=0.213
L6=0.083
L7=0.083
L8=0.082
L9=0.0535
L10=0.059

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))
	S1 = [0,0,1,0.150,0.150,0] 
	S2 = [0,1,0,-0.162,0,-0.150]
	S3 = [0,1,0,-0.162,0,0.094] 
	S4 = [0,1,0,-0.162,0,0.307]
	S5 = [1,0,0,0,0.162,-0.260] 
	S6 = [0,1,0,-0.162,0,0.390]
	M=np.array([[0,-1,0,0.390],[0,0,-1,0.401],[1,0,0,0.2155],[0,0,0,1]])
	S = np.array([S1,S2,S3,S4,S5,S6])
	# ==============================================================#
	return M, S

def convert_sbracket(s):
	s_br=np.array([[0,-s[2],s[1],s[3]],[s[2],0,-s[0],s[4]],[-s[1],s[0],0,s[5]],[0,0,0,0]])
	return s_br

"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	print("Forward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)
	M, S = Get_MS()
	s_br_1=convert_sbracket(S[0])
	s_br_2=convert_sbracket(S[1])
	s_br_3=convert_sbracket(S[2])
	s_br_4=convert_sbracket(S[3])
	s_br_5=convert_sbracket(S[4])
	s_br_6=convert_sbracket(S[5])
	
	e_st1=expm(s_br_1*theta1)
	e_st2=expm(s_br_2*theta2)
	e_st3=expm(s_br_3*theta3)
	e_st4=expm(s_br_4*theta4)
	e_st5=expm(s_br_5*theta5)
	e_st6=expm(s_br_6*theta6)

	T=e_st1 @ e_st2 @ e_st3 @ e_st4 @ e_st5 @ e_st6 @ M
	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

	
	yaw_WgripRad=np.pi*yaw_WgripDegree/180.0
	x_cen=xWgrip-L9*np.cos(yaw_WgripRad)
	y_cen=yWgrip-L9*np.sin(yaw_WgripRad)
	z_cen=zWgrip #parallel

	#solve theta1
	theta_a=np.arctan2(y_cen,x_cen)
	d_cen=np.sqrt(y_cen**2+x_cen**2)
	d_cen_to_3end=L6+0.027 #parallel
	theta_b=np.arctan2(d_cen_to_3end,np.sqrt(d_cen**2-d_cen_to_3end**2))
	theta1=theta_a-theta_b

	#solve theta6
	theta6=theta1-yaw_WgripRad+np.pi/2

	#solve x3end,y3end,z3end
	z3end=z_cen+L10+L8
	vector_xy=np.array([[-(L6+0.027)],[-L7],[1]])
	Trans_M=np.array([[np.cos(theta1),-np.sine(theta1),0],\
				 	[np.sin(theta1),np.cos(theta1),0],\
					[0,0,1]])
	trans_pt=Trans_M@vector_xy
	x3end=vector_xy[0][0]
	y3end=vector_xy[1][0]


	# theta1 = 0.0
	# theta2 = 0.0
	# theta3 = 0.0
	# theta4 = 0.0
	# theta5 = 0.0
	# theta6 = 0.0
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)

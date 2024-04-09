#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
L1=0.152
L2=0.120
L3=0.244
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

	return_value[0] = theta1 + np.pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*np.pi)
	return_value[4] = theta5
	return_value[5] = theta6

	print(T)
	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

	
	yaw_WgripRad=np.pi*yaw_WgripDegree/180.0
	x_cen=xWgrip-L9*np.cos(yaw_WgripRad)+0.15
	y_cen=yWgrip-L9*np.sin(yaw_WgripRad)-0.15
	z_cen=zWgrip-0.01 #parallel

	#solve theta1
	theta1_1=np.arctan2(y_cen,x_cen)
	d_a=np.sqrt(y_cen**2+x_cen**2)
	d_b=L6+0.027 
	d_ab=np.sqrt(d_a**2-d_b**2)
	theta1_2=np.arctan2(d_b,d_ab)
	theta1=theta1_1-theta1_2

	#solve theta6
	theta6=theta1-yaw_WgripRad+np.pi/2

	#solve x3end,y3end,z3end
	z3end=z_cen+L10+L8
	d_xy=np.array([-L7,-(L6+0.027),1])
	Trans_M=np.array([[np.cos(theta1),-np.sin(theta1),x_cen],\
				 	[np.sin(theta1),np.cos(theta1),y_cen],\
					[0,0,1]])
	trans_pt=Trans_M@d_xy
	x3end=trans_pt[0]
	y3end=trans_pt[1]
	
	#solve theta3
	joint_1 = np.array([0,0,L1])
	d_c = (np.array([x3end,y3end,z3end]) - joint_1)
	d_c = np.linalg.norm(d_c)
	
	theta3_1 = np.arccos((d_c**2 - L3**2 - L5**2)/(-2*L3*L5))
	theta3 = np.pi - theta3_1

	#solve theta2
	d_3end=np.sqrt(x3end**2 + y3end**2)
	theta2_1 = np.arctan((z3end-L1)/ d_3end)
	theta2_2 = np.arcsin(np.sin(theta3_1) * L5 / d_c)
	theta2 = -(theta2_1 + theta2_2)
	
	#solve theta4
	theta4 = -(theta2 + theta3)

	theta5 = -np.pi/2

	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)

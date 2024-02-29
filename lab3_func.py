#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	# S1 = [0,0,1,0.150,0.150,0] 
	# S2 = [0,1,0,-0.152,0,-0.150]
	# S3 = [0,1,0,-0.152,0,-0.94] 
	# S4 = [0,1,0,-0.152,0,0.307]
	# S5 = [1,0,0,0,0.152,-0.260] 
	# S6 = [0,1,0,-0.152,0,0.390]
	S1 = [0,0,1,150,150,0] 
	S2 = [0,1,0,-162,0,-150]
	S3 = [0,1,0,-162,0,94] 
	S4 = [0,1,0,-162,0,307]
	S5 = [1,0,0,0,162,-260] 
	S6 = [0,1,0,-162,0,390]
	M=np.array([[0,-1,0,393],[0,0,-1,400.35],[1,0,0,218.1],[0,0,0,1]])
	S = np.array([S1,S2,S3,S4,S5,S6])
    #x,y,z      245, 410, 335
    #angle      0 -45 50 -45 0 -90
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

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	M,S=Get_MS()
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

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value

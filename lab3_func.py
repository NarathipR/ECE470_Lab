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

	#Mearsurement
	M=np.array([[0,-1,0,0.393],[0,0,-1,0.40035],[1,0,0,0.2181],[0,0,0,1]])
	#543.0-150
	#250.35+150
	#208.1+10


	# ==============================================================#
	return M, S

def convert_sbracket(s):
	s_br=np.array([[0,-s[2],s[1],s[3]],[s[2],0,-s[0],s[4]],[-s[1],s[0],0,s[5]],[0,0,0,1]])
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
	
	e_st1=expm(s_br_1*np.radians(theta1))
	e_st2=expm(s_br_2*np.radians(theta2))
	e_st3=expm(s_br_3*np.radians(theta3))
	e_st4=expm(s_br_4*np.radians(theta4))
	e_st5=expm(s_br_5*np.radians(theta5))
	e_st6=expm(s_br_6*np.radians(theta6))

	T=e_st1 @ e_st2 @ e_st3 @ e_st4 @ e_st5 @ e_st6 @ M
	print('T  ',T)
	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value

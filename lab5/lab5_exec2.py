#!/usr/bin/env python

import sys
import copy
import time
import rospy

import numpy as np
from lab5_header import *
from lab5_func import *
from blob_search import *


# ========================= Student's code starts here =========================

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]
target = [204*PI/180.0, -46*PI/180.0, 90*PI/180.0, -124*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Store world coordinates of green and yellow blocks
#xw_yw_G = [xw_yw_G , 30]
#xw_yw_Y = [xw_yw_Y , 30]
xw_yw_G = [ ]
xw_yw_Y = [ ]
xw_yw_G_queue = [ ]
xw_yw_Y_queue = [ ]

bl=[0.39,-0.06,0.035]
tl=[0.15,-0.06,0.035]
br=[0.39,0.347,0.035]
tr=[0.15,0.347,0.035]
# Any other global variable you want to define
# Hints: where to put the blocks?


# ========================= Student's code ends here ===========================

################ Pre-defined parameters and functions no need to change below ################

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    #print('test  ',digital_in_0)
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0
    #print('test2  ',digital_in_0)

"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

################ Pre-defined parameters and functions no need to change above ################
foundObj=False
def gripper_callback(msg):
    #suction param??
    global foundObj
    if msg.AIN0>2:
        foundObj=True
    else:
        foundObj=False
        
def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):
    
    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================
    #global Q
    global foundObj
    error = 0
    start_loc=lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2],0)
    start_loc_top=lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2]+0.1,0)
    end_loc=lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2],0)
    end_loc_top=lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2]+0.1,0)
    
    move_arm(pub_cmd, loop_rate, start_loc_top, vel, accel)
    move_arm(pub_cmd, loop_rate, start_loc, vel, accel)
    gripper(pub_cmd,loop_rate,suction_on)
    time.sleep(1.0)
    move_arm(pub_cmd, loop_rate, start_loc_top, vel, accel)
    if foundObj is False:
        rospy.loginfo("No object found: move to the next one")
        gripper(pub_cmd,loop_rate,suction_off)

    else:
        move_arm(pub_cmd, loop_rate, end_loc_top, vel, accel)
        move_arm(pub_cmd,loop_rate,end_loc,vel,accel)
        gripper(pub_cmd,loop_rate,suction_off)
        time.sleep(1.0)
        move_arm(pub_cmd,loop_rate,end_loc,vel,accel)
        move_arm(pub_cmd, loop_rate, end_loc_top, vel, accel)

    # ========================= Student's code ends here ===========================
    return error


class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")


    def image_callback(self, data):

        global xw_yw_G # store found green blocks in this list
        global xw_yw_Y # store found yellow blocks in this list

        try:
          # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # You will need to call blob_search() function to find centers of green blocks
        # and yellow blocks, and store the centers in xw_yw_G & xw_yw_Y respectively.
        xw_yw_G = blob_search(cv_image, "green")
        xw_yw_Y = blob_search(cv_image, "yellow")
        #blob_search(cv_image, "orange")
        # If no blocks are found for a particular color, you can return an empty list,
        # to xw_yw_G or xw_yw_Y.
        #print("yellow block:",xw_yw_Y,"green block: ",xw_yw_G)
        # Remember, xw_yw_G & xw_yw_Y are in global coordinates, which means you will
        # do coordinate transformation in the blob_search() function, namely, from
        # the image frame to the global world frame.

def select_location(st):
    global tl
    global tr
    global bl
    global br
    val=None
    if st=="tl":
      val=tl  
    elif st=="tr":
      val=tr
    elif st=="bl":
      val=bl  
    elif st=="br":
      val=br  
    return val

"""
Program run from here
"""
def main():
    # global tl
    # global tr
    # global bl
    # global br
    global go_away
    global xw_yw_Y
    global xw_yw_G

    # global variable1
    # global variable2

    # Initialize ROS node
    rospy.init_node('lab5node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 4.0
    accel = 4.0
    move_arm(pub_command, loop_rate, go_away, vel, accel)

    ic = ImageConverter(SPIN_RATE)
    time.sleep(5)

    # ========================= Student's code starts here =========================

    """
    Hints: use the found xw_yw_G, xw_yw_Y to move the blocks correspondingly. You will
    need to call move_block(pub_command, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel)
    """
    xw_yw_G_queue=xw_yw_G
    xw_yw_Y_queue=xw_yw_Y
    # print('xw_yw_G_queue,  ',xw_yw_G_queue)
    # print('xw_yw_Y_queue,  ',xw_yw_Y_queue)
    menu=input("predefined(1) or new_xy(2):  ")

    for pt_y in xw_yw_Y_queue:
        pt_y.append(0.030)
        if menu=='1':
            input_str=input("target position(tr,tl,br,bl):  ")
            target_xw_yw_zw=select_location(input_str)
        elif menu=='2':
            input_str=input("x y:  ")
            values=input_str.split()
            target_xw_yw_zw=[float(values[0]),float(values[1]),0.035]

        #values=input_str.split()
        move_block(pub_command, loop_rate, pt_y, target_xw_yw_zw, vel, accel)
    for pt_g in xw_yw_G_queue:
        pt_g.append(0.030)
        if menu=='1':
            input_str=input("target position(tr,tl,br,bl):  ")
            target_xw_yw_zw=select_location(input_str)
        elif menu=='2':
            input_str=input("x y:  ")
            values=input_str.split()
            target_xw_yw_zw=[float(values[0]),float(values[1]),0.035]
        move_block(pub_command, loop_rate, pt_g, target_xw_yw_zw, vel, accel)
    # # ========================= Student's code ends here ===========================

    move_arm(pub_command, loop_rate, go_away, vel, accel)
    rospy.loginfo("Task Completed!")
    # print("Use Ctrl+C to exit program")
    sys.exit()
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass

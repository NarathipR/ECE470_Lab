#!/usr/bin/env python

import sys
import copy
import time
import rospy

import numpy as np
from lab5_header import *
from lab5_func import *
from blob_search import *
from ultralytics import YOLO
import math

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
obj_dict={}
obj_dict_queue={}

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
    #h_box=0.1
    start_loc=lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2],0)
    start_loc_top=lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2]+0.1,0)
    end_loc=lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2],0)
    end_loc_top=lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2]+0.15,0)
    
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
        self.model = YOLO("yolo-Weights/yolov8n.pt")

# object classes
        self.classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
                    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
                    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
                    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
                    "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
                    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
                    "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
                    "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
                    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
                    "teddy bear", "hair drier", "toothbrush"
                    ]
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

        #cv_image = cv2.flip(raw_image, -1)
        cv_image = raw_image
        #cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)
        x=self.yolo_m(cv_image)
        # You will need to call blob_search() function to find centers of green blocks
        # and yellow blocks, and store the centers in xw_yw_G & xw_yw_Y respectively.
        #xw_yw_G = blob_search(cv_image, "green")
        #xw_yw_Y = blob_search(cv_image, "yellow")
        #blob_search(cv_image, "orange")
        # If no blocks are found for a particular color, you can return an empty list,
        # to xw_yw_G or xw_yw_Y.
        #print("yellow block:",xw_yw_Y,"green block: ",xw_yw_G)
        # Remember, xw_yw_G & xw_yw_Y are in global coordinates, which means you will
        # do coordinate transformation in the blob_search() function, namely, from
        # the image frame to the global world frame.
    def yolo_m(self,img):
        global obj_dict
        results = self.model(img, stream=True)
        for r in results:
            boxes = r.boxes

            for box in boxes:
                # bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

                # put box in cam
                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

                # confidence
                confidence = math.ceil((box.conf[0]*100))/100
                print("Confidence --->",confidence)
                # class name
                cls = int(box.cls[0])
                print(cls)
                bb=[]
                if self.classNames[cls] in obj_dict:
                    bb=obj_dict[self.classNames[cls]]
                    bb.append(box.xyxy[0].cpu().tolist())
                else:
                    bb.append(box.xyxy[0].cpu().tolist())
                obj_dict[self.classNames[cls]]=bb
                print('obj_dict  ',obj_dict)
                print("Class name -->", self.classNames[cls])

                # object details
                org = [x1, y1]
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (255, 0, 0)
                thickness = 2

                cv2.putText(img, self.classNames[cls], org, font, fontScale, color, thickness)

        cv2.imshow('Webcam', img)
        cv2.waitKey(1)
        return obj_dict

Or = 240
Oc = 320
alpha = 0.013349570451068913
beta = 732.268
world=[270.38157653808594, 51.403507232666016]
#world=[262.2084503173828, 51.064043045043945]
# Function that converts image coord to world coord
Or=world[1]
Oc=world[0]
t_x=0.25
t_y=0.08


def IMG2W(col, row):
    x_c=(row-Or)/beta
    y_c=(col-Oc)/beta
    #print('x_c   ',x_c)
    #print('y_c   ',y_c)

    x_w=x_c*np.cos(alpha)+y_c*np.sin(alpha)
    y_w=-x_c*np.sin(alpha)+y_c*np.cos(alpha)
    return [x_w,y_w]

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

def estimate_place_position(pub_command,loop_rate,all_obj,target):
    tx1,ty1,tx2,ty2=target
    mid=int(tx1/2+tx2/2)
    prev_h=int(ty1)
    for obj in all_obj:
        #round up and down
        x1,y1,x2,y2=obj
        x1,y1,x2,y2=int(x1),int(y1),int(x2),int(y2)
        if prev_h+(y2-y1)<=ty2:
            [x,y]=IMG2W(int(x1/2+x2/2), int(y1/2+y2/2))
            start_xw_yw_zw=[x,y,0]

            [x_target,y_target]=IMG2W(mid, prev_h+int(y1/2+y2/2))
            target_xw_yw_zw=[x_target,y_target,0]
            prev_h=prev_h+(y2-y1)

            move_block(pub_command, loop_rate, start_xw_yw_zw, target_xw_yw_zw, 4, 4)

        else:
            break
        
    #known area of target(in pixel)
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
    obj_dict_queue=obj_dict
    # print('xw_yw_G_queue,  ',xw_yw_G_queue)
    # print('xw_yw_Y_queue,  ',xw_yw_Y_queue)
    menu=input("predefined(1) or new_xy(2):  ")
    box_loc=[0.11,0.28,0.43,0.39]
    #where to where
    target_obj="phone"
    if target_obj in obj_dict_queue:
        #put in the position
        #use top and go down
        bb_obj=obj_dict_queue[target_obj]
        estimate_place_position(pub_command,loop_rate,bb_obj,target)
    
    # # ========================= Student's code ends here ===========================

    move_arm(pub_command, loop_rate, go_away, vel, accel)
    rospy.loginfo("Task Completed!")
    # print("Use Ctrl+C to exit program")
    #sys.exit()
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass

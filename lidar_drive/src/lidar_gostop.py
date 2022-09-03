#!/usr/bin/env python

from sensor_msgs.msg import LaserScan
import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))
import random
#sys.path.append("~/xycar_ws/src/hough_drive/src")
#sys.path.insert(0,"/home/nvidia/xycar_ws/src/hough_drive/src/hough_drive.py")
from hough_drive.src.hough_drive import h_start

motor_msg = xycar_motor()
distance = None

image = np.empty(shape=[0])
bridge = CvBridge()
motor = None
img_ready = False 
CAM_FPS = 30
WIDTH, HEIGHT = 640, 480
ROI_ROW = 250
ROI_HEIGHT = HEIGHT - ROI_ROW
L_ROW = ROI_HEIGHT - 120

def callback(data):
    global distance, motor_msg
    distance = data.ranges

def drive(angle,speed):
    global motor_msg
    motor_msg.speed = speed
    motor_msg.angle = angle
    pub.publish(motor_msg)

def drive_stop():
    global motor_msg
    motor_msg.speed = 0
    motor_msg.angle = 0 
    pub.publish(motor_msg)
   
def img_callback(data):
    global image, img_ready
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_ready = True
  
def l_start():

    global image, img_ready, pub, motor

    rospy.init_node('lidar_driver')
    rospy.Subscriber('/scan', LaserScan, callback, queue_size = 1)
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    rospy.sleep(2)
    print ("----- Xycar self driving -----")

    while not image.size == (WIDTH * HEIGHT * 3):
        #print(image.size)
        continue 

     # left 60~120 mid 120 200 right 200 280
    while distance is None:
        continue
 
    while not rospy.is_shutdown():
        while img_ready == False:
           continue
        cv2.imshow("result", image)

        speed=7
        angle=0
        for degree in range(130,190):
            if (0.01 < distance[degree] <= 0.4):
                print('stop - ' ,degree,distance[degree])
                speed=0
                angle=0
                break
        for degree in range(60,130):
            if (0.01 < distance[degree] <= 0.4):
                print('stop -' ,degree,distance[degree])
                speed=6
	        angle=30
                break
        for degree in range(190,280):
            if (0.01 < distance[degree] <= 0.4):
                print('stop - ' ,degree,distance[degree])
                speed=6
	        angle=-30
                break
	h_start()
        drive(angle,speed)
    rospy.spin()
if __name__ == '__main__':
    l_start()


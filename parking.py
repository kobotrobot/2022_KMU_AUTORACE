#! /usr/bin/env python
import rospy
import math
import cv2
import time
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
arData = {"DX": 0.0, "DY": 0.0, "DZ": 0.0,
          "AX": 0.0, "AY": 0.0, "AZ": 0.0, "AW": 0.0}
roll, pitch, yaw = 0, 0, 0
motor_msg = xycar_motor()
motor_pub = None
xycar_msg = None
speed, distance = 0, 0



def init_node():
    global motor_pub, motor_msg, xycar_motor
    rospy.init_node('ar_drive')
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
    motor_pub = rospy.Publisher(
        'xycar_motor', xycar_motor, queue_size=1)


def callback(msg):
    global arData
    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z
        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w


def back_drive():
    global motor_msg, motor_pub, speed, yaw, distance
    speed = -15
    while True:
        show_ARtag()
        dx = arData["DX"]
        dy = arData["DY"]
        angle = -yaw
        if abs(yaw)<2 and abs(dx)<2:
            break
        elif distance > 400:
            break
        elif abs(yaw)<2:
	    angle=-dx
        
        motor_msg.angle = angle
    	motor_msg.speed = speed
        motor_pub.publish(motor_msg)



def show_ARtag():
    global roll, pitch, yaw, distance
    (roll, pitch, yaw) = euler_from_quaternion(
        (arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    img = np.zeros((100, 500, 3))
    img = cv2.line(img, (25, 65), (475, 65), (0, 0, 255), 2)
    img = cv2.line(img, (25, 40), (25, 90), (0, 0, 255), 3)
    img = cv2.line(img, (250, 40), (250, 90), (0, 0, 255), 3)
    img = cv2.line(img, (475, 40), (475, 90), (0, 0, 255), 3)
    point = int(arData["DX"]) + 250
    if point > 475:
        point = 475
    elif point < 25:
        point = 25
    img = cv2.circle(img, (point, 65), 15, (0, 255, 0), -1)
    distance = 100
    distance = math.sqrt(pow(arData["DX"], 2) + pow(arData["DY"], 2))
    cv2.putText(img, str(int(distance))+" pixel", (350, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
    dx_dy_yaw = "DX:"+str(int(arData["DX"]))+" DY:"+str(int(arData["DY"])) \
                + " Yaw:" + str(round(yaw, 1))
    cv2.putText(img, dx_dy_yaw, (20, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))
    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)


if __name__ == '__main__':
    init_node()
    time.sleep(1)
    while not rospy.is_shutdown():
        dx = arData["DX"]
        dy = arData["DY"]
        angle = 50
        speed = 40
        show_ARtag()
        n =(arData["DX"]+arData["DY"])/100
        if(n<1):
	    n=1

        if(abs(yaw)>8):
	    if(arData["DX"]<0):
	        speed =distance/15
	        angle = -((abs(yaw)+20)/n)
	    else:
	        speed =distance/15
	        angle = (abs(yaw)+20)/n
        else:
	    if(abs(yaw)<2):
	        speed=20
	        angle=arData["DX"]
	    else:
	        if(yaw>0):
		    speed=30
		    angle=math.degrees(math.atan2(arData["DX"],arData["DY"]))
	        else:
		
	    	    if(arData["DX"]+arData["DY"]>150):
	                speed=30
	                angle=(math.atan2(arData["DX"],arData["DY"])+(abs(yaw)))*100
	            else:
		        speed=30
	                angle=(math.atan2(arData["DX"],arData["DY"])-(abs(yaw)))*100

        if(distance<72):
	    if(abs(yaw)<3):
	        if(abs(arData["DX"])):
		    speed=0
		    angle=0
    	    else:
	        back_drive()
	
        # print('yaw->{}, dx->{}, distance->{}, angle->{}'.format(math.degrees(yaw), dx, distance, angle))
	motor_msg.angle = angle
	motor_msg.speed = speed
        motor_pub.publish(motor_msg)
    cv2.destroyAllWindows()

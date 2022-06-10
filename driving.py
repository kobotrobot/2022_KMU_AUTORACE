#!/usr/bin/env python
# -*- coding: utf-8 -*-

# =============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
# =============================================
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
import random


# =============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
# =============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

# =============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
# =============================================
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
bridge = CvBridge()
motor = None  # 모터 토픽을 담을 변수

# =============================================
# 프로그램에서 사용할 상수 선언부
# =============================================
CAM_FPS = 30  # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
global left_grad, right_grad, count
count = 0

# =============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 변수에 옮겨 담음.
# =============================================
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

# =============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
# =============================================
def drive(angle, speed):
    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)

def angle():
    global left_grad, right_grad
    if left_grad > -0.68 and left_grad < -0.58:
        return 0
    else:
        return (left_grad + right_grad) * (-40)

# =============================================
# 실질적인 메인 함수
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함.
# =============================================
def start():
    # 위에서 선언한 변수를 start() 안에서 사용하고자 함
    global motor, image, left_grad, right_grad

    # =========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    # =========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

    print("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue

    # =========================================
    # 메인 루프
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서
    # "이미지처리 +차선위치찾기 +조향각결정 +모터토픽발행"
    # 작업을 반복적으로 수행함.
    # =========================================
    tmp1 = []
    tmp2 = []
    while not rospy.is_shutdown():
        # 이미지처리를 위해 카메라 원본이미지를 img에 복사 저장
        img = image.copy()
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        whiteLower = (200, 200, 200)
        whiteUpper = (255, 255, 255)
        mask = cv2.inRange(img_rgb, whiteLower, whiteUpper)
        cv2.imshow('mask', mask)
        img2 = cv2.bitwise_and(img_rgb, img_rgb, mask=mask)
        gray_img = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        blur_img = cv2.GaussianBlur(gray_img, (3,3), 0)
        dst = cv2.Canny(blur_img, 50, 200)
        #print(dst.shape)
        cv2.imshow('canny', dst)

        # 다각형 roi1 지정
        img = np.zeros((480, 640), np.uint8) #height, width
        #print(img.shape)
        roi1_pt = np.array([[45, 360], [0, 430], [310, 430], [310, 360]])
        cv2.fillConvexPoly(img, roi1_pt, (255, 0, 0))
        #print(img.shape)
        roi1 = cv2.bitwise_and(dst, img)
        #print(roi.shape)

	# roi2 
        img = np.zeros((480, 640), np.uint8) #height, width
        #print(img.shape)
        roi2_pt = np.array([[330, 360], [330, 430], [640, 430], [595, 360]])
        cv2.fillConvexPoly(img, roi2_pt, (255, 0, 0))
        #print(img.shape)
        roi2 = cv2.bitwise_and(dst, img)
        #print(roi.shape)

        # Hough 변환(직선 검출)
        
        lines = cv2.HoughLinesP(roi1, 1.0, np.pi / 180, 50, minLineLength=1, maxLineGap=1000)
        tmp1.append(lines)
        #print(tmp1)
        #print("------")
        #print(type(lines))
        #print(lines)
        
        if lines is not None:
            #print(lines)
            roi1_pt1 = (lines[0][0][0], lines[0][0][1])
            roi1_pt2 = (lines[0][0][2], lines[0][0][3])
            cv2.line(image, roi1_pt1, roi1_pt2, (0, 0, 255), 2, cv2.LINE_AA)
            print(" circle roi_pt2", roi1_pt2)
            
        elif lines is None:
            lines = tmp1[-2]
            #pt1 = (lines[0][0][0], lines[0][0][1])
            # pt2 = (lines[0][0][2], lines[0][0][3])
            #cv2.line(image, pt1, pt2, (0, 0, 255), 2, cv2.LINE_AA)
        else: 
            print("fail")
        cv2.circle(image, roi1_pt2, 5, (255, 0, 0), -1) 

        lines2 = cv2.HoughLinesP(roi2, 1.0, np.pi / 180, 50, minLineLength=1, maxLineGap=1000)
        tmp2.append(lines2)
       # print(tmp2)
        #print("------")
        #print(type(lines))
        # print(lines)
        
        if lines2 is not None:
            #print(lines)
            roi2_pt1 = (lines2[0][0][0], lines2[0][0][1])
            roi2_pt2 = (lines2[0][0][2], lines2[0][0][3])
            cv2.line(image, roi2_pt1, roi2_pt2, (0, 0, 255), 2, cv2.LINE_AA)
            print(" circle ro2_pt1", roi2_pt1)
            # cv2.circle(image, int(lines2[0][0][2], int(lines2[0][0][3]), (255, 0, 0), -1)
        elif lines2 is None:
            lines2 = tmp2[-2]
            #pt1 = (lines[0][0][0], lines[0][0][1])
            # pt2 = (lines[0][0][2], lines[0][0][3])
            #cv2.line(image, pt1, pt2, (0, 0, 255), 2, cv2.LINE_AA)
        else: 
            print("fail")
        cv2.circle(image, roi2_pt1, 5, (255, 0, 0), -1)

        # cv2.LINE_AA 뭔지 모름
        cv2.line(image, (0, 360), (640, 360), (0, 255, 0), 1, cv2.LINE_4)
	#print(2)
        # 좌표 그리기
        # rec

        # cv2.circle(image, (105, 360), 5, (0, 255, 255), -1)
        # cv2.circle(image, (535, 360), 5, (0, 255, 255), -1)
	#print(3)

        # 가운데 구하기
        if lines2 is not None and lines is not None:
            x_pt = (lines2[0][0][0] + lines[0][0][2]) / 2 - 320
            y = 180
            angle = atan2(y,x_pt)
            print("x_pt", x_pt)
            #cv2.circle(image, (int(x_pt), y), (255, 255, 0), -1)
            speed = 25
            if x_pt <=0:
                angle = -angle
        elif lines2 is None and lines is not None:
            lines2 = tmp2[-2]
            x_pt = 320 - lines[0][0][2]
            y = 60
            angle = 32*atan2(y,x_pt)
            speed = 10
            if x_pt <= 0:
                angle = -angle
        elif lines2 is not None and lines is None:
            lines = tmp1[-2]
            x_pt = lines2[0][0][0] - 320
            
            y = 60
            angle = -32*atan2(y,x_pt)
            speed = 10
            
            if x_pt <= 0:
                angle = -angle
        else:
            angle = 0
            speed = 10
        print(angle)
        print(x_pt)
        
        ## cv2.circle(image, (x_pt+320, 360), 5, (255, 0, 0), -1)
	
        # 조향각 구하기(-50-~ +50)
         # left gradient 그리기
        #gradient = float((lines[0][0][1] - lines[0][0][3])) / float((lines[0][0][0] - lines[0][0][2]))
        #if gradient < 0:
        #     left_grad = gradient 
        #left_grad = float((lines[0][0][1] - lines[0][0][3])) / float((lines[0][0][0] - lines[0][0][2]))
        #     print(left_grad)
        #else: gradient > 0:
        #     right_grad = gradient
             
       
        #right gradient 그리기
        # right_grad = float((lines[0][0][1] - lines[0][0][3])) / float((lines[0][0][0] - lines[0][0][2]))
        
        #if 
        # n2 = float(lines[0][0][1] - (right_grad * lines[0][0][0]))


        # =========================================
        # 핸들조향각 값인 angle값 정하기.
        # 차선의 위치 정보를 이용해서 angle값을 설정함.
        # =========================================

        # 우선 테스트를 위해 직진(0값)으로 설정
        #angle = 0
        # print(angle())

        # =========================================
        # 차량의 속도 값인 speed값 정하기.
        # 직선 코스에서는 빠른 속도로 주행하고
        # 회전구간에서는 느린 속도로 주행하도록 설정함.
        # =========================================

        # 우선 테스트를 위해 느린속도(10값)로 설정
        #speed = 20
        # 디버깅을 위해 모니터에 이미지를 디스플레이
        cv2.imshow("CAM View", img)
        cv2.imshow("roi1", roi1)
        cv2.imshow('roi2', roi2)
        cv2.imshow('+', image)
        cv2.waitKey(1)

        # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
        drive(angle, speed)

# =============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임.
# =============================================
if __name__ == '__main__':
    start()

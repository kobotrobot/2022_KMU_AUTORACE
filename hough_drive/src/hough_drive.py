#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
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

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge() # OpenCV 함수를 사용하기 위한 브릿지 
motor = None # 모터 토픽을 담을 변수
img_ready = False # 카메라 토픽이 도착했는지의 여부 표시 

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기
ROI_ROW = 250   # 차선을 찾을 ROI 영역의 시작 Row값 
ROI_HEIGHT = HEIGHT - ROI_ROW   # ROI 영역의 세로 크기  
L_ROW = ROI_HEIGHT - 120  # 차선의 위치를 찾기 위한 기준선(수평선)의 Row값

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 라는 변수에 옮겨 담음.
# 카메라 토픽의 도착을 표시하는 img_ready 값을 True로 바꿈.
#=============================================
def img_callback(data):
    global image, img_ready
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_ready = True
    
#=============================================
# 모터 토픽을 발행하는 함수  
# 입력으로 받은 angle과 speed 값을 
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    global motor
    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

#=============================================
# 실질적인 메인 함수 
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함. 
#=============================================
def h_start():

    global image, img_ready, motor

    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    rospy.init_node('h_drive', anonymous=True)
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    rospy.sleep(2)
    print ("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue
 
    #=========================================
    # 메인 루프 
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서 
    # "이미지처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
    # 작업을 반복적으로 수행함.
    #=========================================
    tmp1 = []
    tmp2 = []
    cnt1 = 0
    cnt2 = 0
    while not rospy.is_shutdown():

        # 카메라 토픽이 도착할때까지 잠시 기다림
        while img_ready == False:
            continue
            
        img = image.copy()  # 이미지처리를 위한 카메라 원본이미지 저장
	
	#img = img-20
        display_img = img  # 디버깅을 위한 디스플레이용 이미지 저장
        img_ready = False  # 카메라 토픽이 도착하면 콜백함수 안에서 True로 바뀜
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	
        #=========================================
        # 원본 칼라이미지를 그레이 회색톤 이미지로 변환하고 
        # 블러링 처리를 통해 노이즈를 제거한 후에 (약간 뿌옇게, 부드럽게)
        # Canny 변환을 통해 외곽선 이미지로 만들기
        #=========================================
	whiteLower = (130, 130,130)
        whiteUpper = (255, 255, 255)
	mask = cv2.inRange(img_rgb, whiteLower, whiteUpper)
	img2 = cv2.bitwise_and(img_rgb, img_rgb, mask=mask)
	gray_img = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
	blur_img = cv2.GaussianBlur(gray_img, (3, 3), 0)
	dst = cv2.Canny(blur_img, 50, 180)
	img = np.zeros((480, 640), np.uint8)
        #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)        #blur_gray = cv2.GaussianBlur(gray,(5,5), 0)
        #edge_img = cv2.Canny(np.uint8(blur_gray), 30, 60)
  	
        # img(원본이미지)의 특정영역(ROI Area)을 잘라내기
        #roi_img = img[ROI_ROW:HEIGHT, 0:WIDTH]
        
        # edge_img의 특정영역(ROI Area)을 잘라내기
        #roi_edge_img = edge_img[ROI_ROW:HEIGHT, 0:WIDTH]
        #cv2.imshow("roi edge img", roi_edge_img)

	roi1_pt = np.array([[0, 320], [0, 430], [200, 430], [200, 320]])
	img_1 = cv2.fillConvexPoly(img, roi1_pt, (255, 0, 0))
	roi1 = cv2.bitwise_and(dst, img_1)
        # ++++
        #roi1 = cv2.dilate(roi1, None, iterations=3)
	img22 = np.zeros((480, 640), np.uint8)
	roi2_pt = np.array([[450, 310], [450, 430], [640, 430], [595, 310]])
        img_2 = cv2.fillConvexPoly(img22, roi2_pt, (255, 0, 0))
	roi2 = cv2.bitwise_and(dst, img_2)        
        # ++++
        #roi2 = cv2.dilate(roi2, None, iterations=3)
      

        # 잘라낸 이미지에서 HoughLinesP 함수를 사용하여 선분들을 찾음
        #all_lines = cv2.HoughLinesP(roi_edge_img, 1, math.pi/180, 50, 50, 20)

        # 선분을 찾을 수 없으면 메인루프 처음으로 가서 다음번 카메라 토픽을 기다렸다 처리함


        # print("After ROI, number of lines : %d" % len(all_lines))

        #line_draw_img = roi_img.copy()
	
	lines = cv2.HoughLinesP(roi1, 1.0, np.pi / 180, 50, minLineLength=1, maxLineGap=1000)
	
        if lines is None:
            cnt1 += 1
            if cnt1 <=10:
                continue
            else:
                pass



        tmp1.append(lines)


	if lines is not None:
            roi1_pt1 = (lines[0][0][0], lines[0][0][1])
            roi1_pt2 = (lines[0][0][2], lines[0][0][3])
            
            cv2.line(image, roi1_pt1, roi1_pt2, (0, 0, 255), 2, cv2.LINE_AA)
            print(" circle roi_pt2", roi1_pt2)
	elif lines is None:
            lines = tmp1[-2]
	    print("line1 none")
            pass
        else:
            print("fail")
	# original roi1_pt2
	cv2.circle(image, roi1_pt2, 5, (255, 0, 0), -1)

	lines2 = cv2.HoughLinesP(roi2, 1.0, np.pi / 180, 50, minLineLength=1, maxLineGap=1000)
        if lines2 is None:
            cnt2 += 1
            if cnt2 <=10:
                continue
            else:
                pass
        tmp2.append(lines2)
        if lines2 is not None:
            roi2_pt1 = (lines2[0][0][0], lines2[0][0][1])
            roi2_pt2 = (lines2[0][0][2], lines2[0][0][3])
            cv2.line(image, roi2_pt1, roi2_pt2, (0, 0, 255), 2, cv2.LINE_AA)
            print(" circle ro2_pt1", roi2_pt1)
        elif lines2 is None:
            lines2 = tmp2[-2]
	    print("line2 none")
            pass
        else:
            print("fail")
	# original (roi2_pt1)
        cv2.circle(image, roi2_pt1, 5, (255, 0, 0), -1)

	# green line 
        cv2.line(image, (0, 330), (640, 330), (0, 255, 0), 1, cv2.LINE_4)

	if lines2 is not None and lines is not None:
            x_pt = (lines2[0][0][0] + lines[0][0][2]) / 2 - 320
            y = 60	
            angle = 8*atan2(y, x_pt)
            speed = 9
            if x_pt <= 0:
                angle = -(1.2)*angle
	    print(angle)
            print("----------------------------")
        # 왼쪽 라인만 검출 되는 경우
        elif lines2 is None and lines is not None:
            lines2 = tmp2[-2]
            x_pt =  lines[0][0][2]
            y = 60
            angle = 80* atan2(y, x_pt)
            speed = 8

            if lines[0][0][2] >= 195:
                y_pt = lines[0][0][3]
                angle = angle + y_pt - 330
	    print(angle)
        # 오른쪽 라인만 검출 되는 경우
        elif lines2 is not None and lines is None:
            lines = tmp1[-2]
            x_pt = 640-lines2[0][0][0] 
            y = 60
            angle = -80*atan2(y, x_pt)
            speed = 8
            if lines2[0][0][0] <= 455:
                y_pt = lines2[0][0][1]
                angle = angle + 330 - y_pt
	    print(angle)
        else:
            angle=0
            speed = 6
	    #print("oo")

        cv2.imshow('jam', roi1)
        cv2.imshow('jam2', roi2)
	#cv2.imshow("img_1", img_1)
	#cv2.imshow("img_2", img_2)
	cv2.imshow('+', image)
	cv2.imshow("mask", mask)
        cv2.waitKey(1)
	drive(angle, speed)
	print(speed)
    rospy.spin()
        
        

#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
#=============================================
if __name__ == '__main__':
    h_start()


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

# =============================================
# 실질적인 메인 함수
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함.
# =============================================

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
    # 뒤에 나오지만, line(차선)을 인식하지 못할 때를 대비하여 즉 line=0이 나올 때를 대비하여 tmp1, tmp2에 저장하여 선이 없을 때를 예외처리 해줄 예정임
    tmp1 = []
    tmp2 = []
    while not rospy.is_shutdown():
        # 이미지처리를 위해 카메라 원본이미지를 img에 복사 저장
        img = image.copy()
        # opencv 는 BGR을 기본 포맷으로 쓰기 때문에 RGB로 바꿔서 이미지를 img_rgb에 저장
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # 흰 색 차선을 검출하기 위해 특정 색상인 흰 색의 하한 값과 상한 값을 지정
        whiteLower = (200, 200, 200)
        whiteUpper = (255, 255, 255)
        # 차선의 흰 색 부분을 검출하기 위해 inRange()를 사용해 흰 색 영역만 검출하고 나머지는 다 검정으로 지정
        mask = cv2.inRange(img_rgb, whiteLower, whiteUpper)
        # image에서 흰 색으로 바뀐 부분만 mask에 저장해서 mask를 화면에 보여줌, 그러면 흰 색 차선만 영상에 띄워짐
        cv2.imshow('mask', mask)
        # bitwise_and()를 통해 mask된 부분만 출력해서 img2에 저장
        img2 = cv2.bitwise_and(img_rgb, img_rgb, mask=mask)
        # img2를 이제 gray로 바꿔서 gray_img에 저장
        gray_img = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        # GaussianBlur()를 사용하여 이미지를 부드럽게 해서 blur_img에 저장, 세번째 argument를 0으로 해서 커널 사이즈 (3, 3)에 맞춰서 시그마를 계산해서 사용
        ######### 가우시안 블러 안 써도 됨........!!!!!!!
        blur_img = cv2.GaussianBlur(gray_img, (3, 3), 0)
        # 흰 색 차선을 검출한 영상에서 경계선만 따기 위해 Canny()를 써서 테두리만 검출 \
        # 경계선을 검출할 때 잡음에 매우 민감하기 때문에 블러 처리를 해서 경계선을 둔감하게 함 \
        # 50은 weak edge를 검출하기 위한 임계값임. 200은 strong edge를 검출하기 위한 임계값임
        dst = cv2.Canny(blur_img, 50, 200)
        # canny()를 사용한 영상을 띄움
        cv2.imshow('canny', dst)

        # 특정 영역만 검출하기 위해 img를 0으로 채워서 검정색 화면으로 바꿔 img에 저장
        img = np.zeros((480, 640), np.uint8)  # height, width
        # [[45, 360], [0, 430], [310, 430], [310, 360]] 범위로 특정 영역(roi1_pt)을 설정함. \
        # 자동차의 주행 화면에서 차선을 검출하기 위해서 화면 아래 부분 왼쪽 roi1_pt로 설정한 것임.
        roi1_pt = np.array([[45, 360], [0, 430], [310, 430], [310, 360]])
        # fillConvexPoly()를 사용하여 위에 점들을 꼭짓점으로 다각형을 그림
        cv2.fillConvexPoly(img, roi1_pt, (255, 0, 0))
        # canny()를 사용한 dst와 img를 합쳐서 roi1에 저장
        roi1 = cv2.bitwise_and(dst, img)

        # 특정 영역만 검출하기 위해 img를 0으로 채워서 검정색 화면으로 바꿔 img에 저장
        img = np.zeros((480, 640), np.uint8)  # height, width
        # print(img.shape)
        # [[330, 360], [330, 430], [640, 430], [595, 360]] 범위로 특정 영역(roi2_pt)을 설정함.
        # 자동차의 주행 화면에서 차선을 검출하기 위해서 화면 아래 부분 오른쪽 roi2_pt로 설정한 것임.
        roi2_pt = np.array([[330, 360], [330, 430], [640, 430], [595, 360]])
        # fillConvexPoly()를 사용하여 위에 점들을 꼭짓점으로 다각형을 그림
        cv2.fillConvexPoly(img, roi2_pt, (255, 0, 0))
        # canny()를 사용한 dst와 img를 합쳐서 roi2에 저장
        roi2 = cv2.bitwise_and(dst, img)

        # =========================================
        # HoughLinesP 변환(직선 검출)
        # 허프 변환은 모든 점에 대해서 계산하기 때문에 시간이 많이 소요되어 확률 허프 변환을 사용하여 이전 허프변환을 최적화함.
        # 모든 점을 대상으로 하는 것이 아니라 임의의 점을 이용하여 직선을 찾음.
        # roi1에 대해 허프변환을 하여 직선을 찾음.
        # 2번째 파라미터는 r값의 범위임
        # 3번째 파라미터는 세타 값의 범위임
        # 4번쨰 파라미터는 직선으로 판단할 최소한의 점을 50개로 지정함.
        # 5번째 파라미터는 선으로 인정할 최소 길이를 1로 설정하였고, 6번쨰 파라미터는 선으로 판단할 최대 간격을 1000으로 설정함.
        # =========================================
        lines = cv2.HoughLinesP(roi1, 1.0, np.pi / 180, 50, minLineLength=1, maxLineGap=1000)
        # tmp1에 검출된 선을 추가함.
        tmp1.append(lines)
        # 차선이 검출된다면 선의 양 끝 꼭짓점의 길이를 구하고 영상에 선을 그림.
        if lines is not None:
            # roi1_pt1은 왼쪽 roi1에 아래쪽 x,y 좌표이고 roi1_pt2는 위쪽 x,y 좌표임
            roi1_pt1 = (lines[0][0][0], lines[0][0][1])
            roi1_pt2 = (lines[0][0][2], lines[0][0][3])
            # roi1_pt1 과 roi1_pt2를 잇는 선을 그림, 색은 빨간색으로 설정하고 cv2.LINE_AA는 픽셀이 깨져서 발생하는 계단 현상을 최소화하기 위해 타입을 선택함.
            cv2.line(image, roi1_pt1, roi1_pt2, (0, 0, 255), 2, cv2.LINE_AA)
            print(" circle roi_pt2", roi1_pt2)
        # 차선이 검출되지 않았다면, tmp1에 저장되어 있던 -2 인덱스 값을 저장함
        elif lines is None:
            lines = tmp1[-2]
        else:
            print("fail")
        # 직선이 생기는 방향 위쪽에 꼭짓점 원을 그림.
        cv2.circle(image, roi1_pt2, 5, (255, 0, 0), -1)

        # roi2 영역의 직선 검출
        lines2 = cv2.HoughLinesP(roi2, 1.0, np.pi / 180, 50, minLineLength=1, maxLineGap=1000)
        tmp2.append(lines2)
        # 차선이 검출된다면 선의 양 끝 꼭짓점의 길이를 구하고 영상에 선을 그림.
        if lines2 is not None:
            # roi2_pt1은 오른쪽 위쪽 x, y 좌표이고, roi2_pt2는 아래쪽 x,y 좌표임
            roi2_pt1 = (lines2[0][0][0], lines2[0][0][1])
            roi2_pt2 = (lines2[0][0][2], lines2[0][0][3])
            # roi2_pt1 과 roi2_pt2를 잇는 선을 그림, 색은 빨간색으로 설정하고 cv2는 LINE_AA는 픽셀이 깨져서 발생하는 계단 현상을 최소화하기 위해 타입을 선택함.
            cv2.line(image, roi2_pt1, roi2_pt2, (0, 0, 255), 2, cv2.LINE_AA)
            print(" circle ro2_pt1", roi2_pt1)
        elif lines2 is None:
            lines2 = tmp2[-2]
        else:
            print("fail")
            # 직선이 생기는 방향 위쪽 꼭짓점 원을 그림
        cv2.circle(image, roi2_pt1, 5, (255, 0, 0), -1)
        # 차량의 roi 범위를 영상에 표현하기 위해 선으로 그림.
        cv2.line(image, (0, 360), (640, 360), (0, 255, 0), 1, cv2.LINE_4)

        # =========================================
        # 핸들조향각 값인 angle값 정하기.
        # 차선의 위치 정보를 이용해서 angle값을 설정함.
        # =========================================
        # =========================================
        # 차량의 속도 값인 speed값 정하기.
        # 직선 코스에서는 빠른 속도로 주행하고
        # 회전구간에서는 느린 속도로 주행하도록 설정함.
        # =========================================
        # 우선 테스트를 위해 느린속도(10값)로 설정
        # speed = 20

        if lines2 is not None and lines is not None:
            x_pt = (lines2[0][0][0] + lines[0][0][2]) / 2 - 320
            y = 180
            angle = atan2(y, x_pt)
            speed = 50
            if x_pt <= 0:
                angle = -angle
        elif lines2 is None and lines is not None:
            lines2 = tmp2[-2]
            x_pt = 320 - lines[0][0][2]
            y = 60
            angle = 32 * atan2(y, x_pt)
            speed = 30
            if x_pt <= 0:
                angle = -angle
        elif lines2 is not None and lines is None:
            lines = tmp1[-2]
            x_pt = lines2[0][0][0] - 320
            y = 60
            angle = -32 * atan2(y, x_pt)
            speed = 30
            if x_pt <= 0:
                angle = -angle
        else:
            angle = 0
            speed = 25

        # 우선 테스트를 위해 직진(0값)으로 설정
        # angle = 0

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

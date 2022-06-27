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

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
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

# 만약 목표지점에 도달했는데 yaw값이 맞지 않거나 목표사각형을 넘어가게 되면 실행하는 코드
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
 	# 후진을 한 후에 다시 정렬을 하는 함수
        # 후진을 하면서 yaw 값을 맞추면서 후진을 함.
        # 그리고 yaw 값이 맞춰질 때 까지 후진을 하거나 다시 직진을 하면서 yaw 같을 맞출 수 있을 정도로 후진을 하면 다시 직진하면서 yaw 값을 맞춤.
        
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
	# n 이라는 값은 dx,dy 값의 변화에 따라 가중치를 바꿔 주기 위해서 만든 변수임.
        # 그리고 만약 dx,dy를 활용하여 가중치를 쓰는데 n의 값이 0보다 작으면 가중치가 나누었을 경우에는 값이 증가하는 식으로 바뀌기 때문에 0보다 작을 때는 1로 고정을 해줌.

        if(abs(yaw)>8):
	    if(arData["DX"]<0):
	        speed =distance/15
	        angle = -((abs(yaw)+20)/n)
 	    # angle에 -를 준 이유는 x가 0보다 작을 때 각도가 음수 방향으로 움직여야 하기 때문임.
            # yaw값에 20을 더한 후, n 으로 나눈 이유는 가중치를 적절하게 주어야 어느 위치에서든지 각도 조정을 잘하면서 움직이는데 20이 가장 적정치였음.
	    else:
	        speed =distance/15
	        angle = (abs(yaw)+20)/n
 	# speed를 거리에 나눈 이유는 거리가 가까워 질수록 좀더 정교한 움직임이 필요하기에 나눔.
        # 그리고 여러가지 수로 테스트를 해봤을 때, 15가 가장 적정값이여서 15로 나눔.
        else:
	    if(abs(yaw)<2):
	        speed=20
	        angle=arData["DX"]
	    # yaw 값이 절대값 기준으로 2~8사이인 경우에는 dx값 그대로 앵글값을 주면 dx가 점점 0을 향해 가면서
            # dx가 0으로 가면 yaw 값도 점점0에 가까워짐.
	    else:
	        if(yaw>0):
		    speed=30
		    angle=math.degrees(math.atan2(arData["DX"],arData["DY"]))
		# x,y좌표를 아크탄젠트를 활용해 각도를 라디안으로 구하고 디그리값으로 바꿔서 넣어줌.
                # 이 시뮬레이터에서 사용하는 앵글이 디그리 값인 것 같아서 이런 식으로 함.
	        else:
		
	    	    if(arData["DX"]+arData["DY"]>150):
	                speed=30
	                angle=(math.atan2(arData["DX"],arData["DY"])+(abs(yaw)))*100
	            else:
		        speed=30
	                angle=(math.atan2(arData["DX"],arData["DY"])-(abs(yaw)))*100
    # yaw 값을 정교하게 맞춰주는 작업
    # 만약 dx,dy값을 더한 값이 넉넉하게 150이상일 경우에는 dx,dy의 아크탄젠트 값을 yaw 값에 더해서 100의 값을 곱하여 yaw 값을 정교하게 맞추도록 하였고,
    # 그렇지 않을 경우에는 아크탄젠트 값에서 yaw값을 빼주면서 yaw값을 정교하게 맞출 수 있도록 함.

        # 거리가 72안으로 들어와 있고 yaw 값이 3이내 이면 자동차가 주차 구역 안에 있다고 판단하여 정지
        # 아닐 경우에는 back_drive()사용
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

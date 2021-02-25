#!/usr/bin/python

import cv2, rospy, time
import numpy as np
import math

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

bridge = CvBridge()
control_msg = Twist()

cv_image = np.empty(shape=[0])
x, y, theta = 0, 0, 0

def callback(img_data):
	global bridge
	global cv_image
	cv_image = bridge.imgmsg_to_cv2(img_data, 'bgr8')

def control_msg_publish(linear_x, angular_z):
	control_msg.linear.x = linear_x
	control_msg.linear.y = 0
	control_msg.linear.z = 0
	control_msg.angular.x = 0
	control_msg.angular.y = 0
	control_msg.angular.z = angular_z

	pub.publish(control_msg)

def pose_callback(pose_data):
	global x 
	global y
	global the

	x = pose_data.x
	y = pose_data.y
	the = pose_data.theta

def angular_detection(angular):
	while angular >= np.pi:
		angular = angular - 2.0*np.pi
		
	while angular < -np.pi:
		angular = angular + 2.0*np.pi

	return angular

def move_goal(x_move, y_move):
	dis = abs(math.sqrt(((x_move-x) ** 2) + ((y_move-y) ** 2)))
	linear_value = 0.5

	linear_spd = dis * linear_value

	angular_value = 4
	angular_goal_the = math.atan2(y_move-y, x_move-x)

	angle = angular_detection(angular_goal_the - the)

	angular_spd= angle * angular_value

	control_msg_publish(linear_spd, angular_spd)

	if (dis < 0.5):
		control_msg_publish(0,0)

if __name__ == "__main__":
	rospy.init_node("cam_node")
	rospy.Subscriber("/usb_cam/image_raw", Image, callback)
	rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 1 )
	time.sleep(1)

	while not rospy.is_shutdown():
		#image processing
		img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

		#blue
		lower_color = (100, 100, 100)
		upper_color = (140, 255, 255)
		img_mask = cv2.inRange(img_hsv, lower_color, upper_color)

		img_result = cv2.bitwise_and(cv_image, cv_image, mask = img_mask)

		k = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
		img_result = cv2.morphologyEx(img_result, cv2.MORPH_CROSS, k)

		#(goal_x, goal_y)
		nzero = np.nonzero(img_result)
		nzero_x = np.mean(nzero[1]) / 60

		#nzero_y = ((min(nzero[0]) + max(nzero[0]))) / 2
		nzero_y = np.mean(nzero[0]) / 2
		nzero_y_2 = (-0.023 * nzero_y) + 11 	# 11:11=640:480

		if nzero != 0:
			move_goal(nzero_x, nzero_y_2)	#move_goal(10.0,10.0)
		else:
			print("ERROR")

		cv2.rectangle(img_result, (min(nzero[1]), min(nzero[0])), (max(nzero[1]), max(nzero[0])), (2,255,0),2 )

		img_line = cv2.line(img_result, (0,160), (640,160),(255,255,255),1)
		img_line = cv2.line(img_result, (0,320), (640,320),(255,255,255),1)
		img_line = cv2.line(img_result, (213,0), (213,480),(255,255,255),1)
		img_line = cv2.line(img_result, (426,0), (426,480),(255,255,255),1)

		cv2.imshow("camera", img_line)
		

		if cv2.waitKey(1) & 0xff == ord("q"):

			break

	cv2.destroyAllWindows()

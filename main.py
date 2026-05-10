# sigh
import sys
sys.path.append('/home/oscar/oscar_dev/env/lib/python3.13/site-packages')

# drive
from gpiozero import Motor
from gpiozero import Servo
from gpiozero import DistanceSensor

# vision
import cv2 as cv
from pupil_apriltags import Detector

# general
from time import time
from time import sleep
import numpy
import random

########### ACTUATORS & SENSORS
# motors
front_left_motor = Motor(forward=25, backward=24, enable=18)
back_left_motor = Motor(forward=7, backward=8, enable=23)
back_right_motor = Motor(forward=22, backward=4, enable=11)
front_right_motor = Motor(forward=9, backward=10, enable=6)
turn_factor = 0.9

# servos
left_servo = Servo(13)
right_servo = Servo(1)

# us sensors
front_us = DistanceSensor(trigger=21, echo=19)
left_us = DistanceSensor(trigger=0, echo=5)
right_us = DistanceSensor(trigger=14, echo=15)
back_us = DistanceSensor(trigger=20, echo=16)
limit = 25

def servo_up():
	left_servo.value = 0.9
	right_servo.value = 1

def servo_detach():
	left_servo.detach()
	right_servo.detach()
	 
def oscar_stop():
	front_left_motor.stop()
	front_right_motor.stop()
	back_left_motor.stop()
	back_right_motor.stop()

def oscar_forward(speed):
	front_left_motor.forward(speed)
	front_right_motor.forward(speed)
	back_left_motor.forward(speed)
	back_right_motor.forward(speed)

def oscar_forward_limited(distance):
	t=1
	front_left_motor.forward(0.8)
	front_right_motor.forward(0.8)
	back_left_motor.forward(0.8)
	back_right_motor.forward(0.8)
	sleep(t)
	oscar_stop()
	
def oscar_backward(speed):
	front_left_motor.backward(speed)
	front_right_motor.backward(speed)
	back_left_motor.backward(speed)
	back_right_motor.backward(speed)

def oscar_point_left(speed, deg):
	if (deg != "inf"):
		t = (turn_factor*deg)/90
		front_left_motor.backward(speed)
		front_right_motor.forward(speed)
		back_left_motor.backward(speed)
		back_right_motor.forward(speed)
		sleep(t)
		oscar_stop()
	elif(deg == "inf"):
		front_left_motor.backward(speed)
		front_right_motor.forward(speed)
		back_left_motor.backward(speed)
		back_right_motor.forward(speed)
	else:
		print("Invalid degree measure input pointing right.")

def oscar_point_right(speed, deg):
	if (deg != "inf"):
		t = (turn_factor*deg)/90
		front_left_motor.forward(speed)
		front_right_motor.backward(speed)
		back_left_motor.forward(speed)
		back_right_motor.backward(speed)
		sleep(t)
		oscar_stop()
	elif (deg == "inf"):
		front_left_motor.forward(speed)
		front_right_motor.backward(speed)
		back_left_motor.forward(speed)
		back_right_motor.backward(speed)
	else:
		print("Invalid degree measure input pointing right.")

########### APRILTAG
fx = 4208
fy = 3670
cx = 322.6
cy = 236.5

at_detector = Detector(families='tag36h11')
at_detector_params = (fx, fy, cx, cy)
tag_size = 0.762

def at_check(frame):
	gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
	
	results = at_detector.detect(
		gray,
		estimate_tag_pose=True,
		camera_params=at_detector_params,
		tag_size=tag_size)
	
	if results:
		return "found"
	else:
		return "not found"
		
def at_get_delta_x(frame):
	gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

	results = at_detector.detect(
		gray, 
		estimate_tag_pose=True,
		camera_params=at_detector_params,
		tag_size=tag_size)

	for r in results:
		delta_x = r.pose_t[0]
		return delta_x
	
'''
def at_get_h(frame):
	gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

	results = at_detector.detect(
		gray, 
		estimate_tag_pose=True,
		camera_params=at_detector_params,
		tag_size=tag_size)

	for r in results:
		h = r.pose_t[1]
		return h
'''

def at_get_corners(frame):
	gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

	results = at_detector.detect(
		gray, 
		estimate_tag_pose=True,
		camera_params=at_detector_params,
		tag_size=tag_size)

	if results:
		return results[0].corners[0],results[0].corners[2]
	else:
		return (1920,0),(0,1080)

'''
def at_get_theta(delta_x, h):
	theta_rad = numpy.arcsin(delta_x/h)
	theta_deg = numpy.rad2deg(theta_rad)
	return theta_deg
'''
		

########### BLOB
bDetector_params = cv.SimpleBlobDetector.Params()

bDetector_params.filterByColor = True
bDetector_params.blobColor = 0

# thresholding (recoloring pixels to make them easier to differentiate between)
bDetector_params.minThreshold = 10
bDetector_params.maxThreshold = 200

# circularity
bDetector_params.filterByCircularity = True
bDetector_params.minCircularity = 0.5

# circularity
bDetector_params.filterByArea = True
bDetector_params.minArea = 100
bDetector_params.maxArea = 10000000

# uninterested
bDetector_params.filterByConvexity = False
bDetector_params.filterByInertia = False

bDetector = cv.SimpleBlobDetector_create(bDetector_params)

def find_blobs(frame, corner1, corner2):
	x_start = int(corner2[0])
	x_end = int(corner1[0])
	y_start = int(corner1[1])
	y_end = int(corner2[1])

	if (x_start != 0):
		cv.rectangle(frame,(x_start,y_start),(x_end,y_end),(255,255,255),-1)
	
	blob = bDetector.detect(frame)
	
	output = cv.drawKeypoints(frame, 
							  blob, 
							  numpy.array([]), 
							  (0, 0, 0),
							  cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	
	cv.imshow("Blobs Detected", output)

########## CAMERA SETUP
camera = cv.VideoCapture(0)

if not camera.isOpened():
	print("Cannot open camera.")
	exit()

########## MAIN LOOP
looping = True
check = "not found"

# initial check
check = at_check()
if (check != "found"):
	print("Tag not found.")
	looping = False

# main loop
while looping:
	ret,frame = camera.read()
	if not ret:
		print("Camera returning no input.")
		looping = False
	
	distance_front = front_us.distance * 100
	distance_back = back_us.distance * 100
	distance_left = left_us.distance * 100
	distance_right = right_us.distance * 100

	# final navigation and cleaning
	if (d_x > -1 and d_x < 1 and distance_front < limit):
		print("Arrived")
		# check if dirty
		# if dirty, clean
		# if not, don't
		# when cleaned do a twirl

	# navigation to tag
	elif (distance_front >= limit):
		d_x = at_get_delta_x()

		if (d_x >= -1):
			oscar_point_right(1, 90)
			oscar_forward_limited(d_x)
			oscar_point_left(1, 90)

		elif (d_x <= 1):
			oscar_point_left(1,90)
			oscar_forward_limited(d_x)
			oscar_point_right(1, 90)

		elif (d_x > -1 and d_x < 1):
			oscar_forward(0.8)

		else:
			print("Oscar is lost.")
		
		sleep(1)
		oscar_stop()
		sleep(1)
		
	key = cv.waitKey(100)
	if key == 13:
		looping = False

camera.release()
cv.destroyAllWindows()

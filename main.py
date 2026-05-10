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
	
def oscar_backward(speed):
	front_left_motor.backward(speed)
	front_right_motor.backward(speed)
	back_left_motor.backward(speed)
	back_right_motor.backward(speed)

def oscar_point_left(speed, deg):
	if (deg != "inf"):
		t = (0.77*deg)/90
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
		t = (0.77*deg)/90
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
	
def oscar_spin():
	t = 0.77*4
	oscar_point_left(1)
	sleep(t)
	oscar_stop()
	sleep(0.5)
	oscar_point_right(1)
	sleep(t)
	oscar_stop()

########### APRILTAG
fx = 500
fy = 500
cx = 500
cy = 500

at_detector = Detector(families='tag36h11')
at_detector_params = (fx, fy, cx, cy)
tag_size = 0.762

def at_get_delta_x(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    results = at_detector.detect(
        gray, 
        estimate_tag_pose=True,
        camera_params=at_detector_params,
        tag_size=tag_size)

    for r in results:
        x_dist = r.pose_t[0]
        return x_dist
    
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

def at_get_theta(delta_x, h):
    theta_rad = numpy.arcsin(delta_x/h)
    theta_deg = numpy.rad2deg(theta_rad)
    return theta_deg

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

def at_poll(frame):
	poll = []
	start = time()
	while ((time() - start) < 1):
		poll.append(at_check(frame))
		sleep(0.1)
		
	for result in poll:
		if (result == "found"):
			return "found"
        

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
while looping:
	ret,frame = camera.read()
	if not ret:
		print("Camera returning no input.")
		looping = False
	
	# check if there are objects
	distance_front = front_us.distance * 100
	distance_back = back_us.distance * 100
	distance_left = left_us.distance * 100
	distance_right = right_us.distance * 100
	
	# go forward if there are no obstacles
	if (distance_front >= limit):
		servo_detach()
		oscar_forward(1)

	# if there is an obstacle
	else:
		# stop and assess what would be an ideal path
		oscar_stop()
		sleep(1)
		
		if (distance_left > distance_right):
			oscar_point_left(1, 90)
			at_poll(frame)
			sleep(1)
		elif (distance_right < distance_left):
			oscar_point_right(1, 90)
			at_poll(frame)
			sleep(1)
		elif (distance_right == distance_left):
			guess = random.randint(0,1)
			if (guess == 0):
				oscar_point_left(1, 90)
				sleep(1)
			else:
				oscar_point_right(1, 90)
				sleep(1)
		else:
			print("Oscar is lost.")

	key = cv.waitKey(100)
	if key == 13:
	    looping = False

camera.release()
cv.destroyAllWindows()

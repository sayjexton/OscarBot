# drive
from gpiozero import Motor
from gpiozero import Servo
from gpiozero import DistanceSensor

# vision
import cv2 as cv
from pupil_apriltags import Detector
import numpy

# general
from time import sleep

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

def servo_up():
	left_servo.value = 0.9
	right_servo.value = 1

def servo_detach():
	left_servo.detach()
	right_servo.detach()

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

def oscar_point_left(speed):
	front_left_motor.backward(speed)
	front_right_motor.forward(speed)
	back_left_motor.backward(speed)
	back_right_motor.forward(speed)

def oscar_point_right(speed):
	front_left_motor.forward(speed)
	front_right_motor.backward(speed)
	back_left_motor.forward(speed)
	back_right_motor.backward(speed)

def oscar_stop():
	front_left_motor.stop()
	front_right_motor.stop()
	back_left_motor.stop()
	back_right_motor.stop()

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
    print()

camera.release()
cv.destroyAllWindows()

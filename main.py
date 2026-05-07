import cv2 as cv
from pupil_apriltags import Detector
import numpy

# camera parameters
fx = 500
fy = 500
cx = 500
cy = 500

# initializing detector
at_detector = Detector(families='tag36h11')
camera_params = (fx, fy, cx, cy)
tag_size = 0.762

# find offset from center of camera
def get_offset(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    results = at_detector.detect(
        gray, 
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=tag_size)

    for r in results:
        x_dist = r.pose_t[0]
        print("Distance from Center: ", str(x_dist), "m")
        return x_dist

# detection loop
camera = cv.VideoCapture(0)
looping = True

if not camera.isOpened():
    print("Cannot open camera.")
    exit()

while looping:
    ret,frame = camera.read()
    if not ret:
        print("Camera returning no input.")
        break

    # break loop if return key pressed
    key = cv.waitKey(100)
    if key == 13:
        looping = False
    
    get_offset(frame)
    cv.imshow('Camera Feed', frame)

camera.release()
cv.destroyAllWindows()
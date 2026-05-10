import cv2 as cv
from pupil_apriltags import Detector
import numpy
from time import sleep

# ----- APRILTAG DETECTOR -----
fx = 500
fy = 500
cx = 500
cy = 500

at_detector = Detector(families='tag36h11')
at_detector_params = (fx, fy, cx, cy)
tag_size = 0.762

# ----- BLOB DETECTOR -----
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

# ----- FIND OFFSET ------
def at_get_delta_x(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    results = at_detector.detect(
        gray, 
        estimate_tag_pose=True,
        camera_params=at_detector_params,
        tag_size=tag_size)

    for r in results:
        delta_x = r.pose_t[0]
        print("Distance from Center: ", str(delta_x), "m")
        return delta_x
    
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

# ----- FIND CORNER LOCATIONS -----
def get_corners(frame):
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
    
# ----- FIND BLOBS -----
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

# ----- DETECTION LOOP ------
camera = cv.VideoCapture(0)
looping = True

if not camera.isOpened():
    print("Cannot open camera.")
    exit()

while looping:
    ret,frame = camera.read()
    if not ret:
        print("Camera returning no input.")
        looping = False

    print(at_get_h(frame))
    sleep(0.5)

    cv.imshow('Camera Feed', frame)

    #corner1,corner2 = get_corners(frame)
    #find_blobs(frame,corner1,corner2)
    
    #print("x: ", corner2[0],corner1[0])
    #print("y: ", corner1[1],corner2[1])
    
    # break loop if return key pressed
    key = cv.waitKey(100)
    if key == 13:
        looping = False
    
    print(at_get_delta_x(frame))

camera.release()
cv.destroyAllWindows()

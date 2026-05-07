import cv2 as cv
import numpy

bDetector_params = cv.SimpleBlobDetector.Params()

bDetector_params.filterByColor = True
bDetector_params.blobColor = 0

# thresholding (recoloring pixels to make them easier to differentiate between)
bDetector_params.minThreshold = 10
bDetector_params.maxThreshold = 200

# circularity
bDetector_params.filterByCircularity = True
bDetector_params.minCircularity = 0.5

# uninterested
bDetector_params.filterByArea = False
bDetector_params.filterByConvexity = False
bDetector_params.filterByInertia = False

# initialize detector
bDetector = cv.SimpleBlobDetector_create(bDetector_params)

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
        looping = False

    # break loop if return key pressed
    key = cv.waitKey(100)
    if key == 13:
        looping = False

    blobs = bDetector.detect(frame)
    output = cv.drawKeypoints(frame, 
                              blobs, 
                              numpy.array([]), 
                              (0, 0, 0),
                              cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    cv.imshow("Blobs Detected", output)
    
camera.release()
cv.destroyAllWindows()
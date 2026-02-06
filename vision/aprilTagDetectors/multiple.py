import cv2
from dt_apriltags import Detector

# read in the image
filePath = 'multiple.webp'
img = cv2.imread(filePath)

# show original image
#cv2.imshow('img',img)
#cv2.waitKey(0)

# setup the detector(didn't really read what it's all about)
at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

#detector has to work with greyscale i think
grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#makes a list of all the apriltags it finds
detections = at_detector.detect(grey)

final = img

#go through and draw a box around each apriltag it finds
for det in detections:
    x1 = int(det.corners[0][0])
    y1 = int(det.corners[0][1])

    x2 = int(det.corners[2][0])
    y2 = int(det.corners[2][1])

    final = cv2.rectangle(final,(x1,y1),(x2,y2),(0,255,0),5)

cv2.imshow('final',final)
cv2.waitKey(0)
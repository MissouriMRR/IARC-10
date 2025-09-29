import cv2
from dt_apriltags import Detector

# read in the image
filePath = 'apriltag.png'
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

#since we only have one, i just have it print off the corners of the first(and only) apriltag detected
print(detections[0].corners)

x1 = int(detections[0].corners[0][0])
y1 = int(detections[0].corners[0][1])

x2 = int(detections[0].corners[2][0])
y2 = int(detections[0].corners[2][1])

withBox = cv2.rectangle(img,(x1,y1),(x2,y2),(0,255,0),5)

cv2.imshow('final',withBox)
cv2.waitKey(0)
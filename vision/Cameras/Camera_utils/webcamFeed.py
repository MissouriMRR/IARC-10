import cv2
from dt_apriltags import Detector

webcam = cv2.VideoCapture(0)

at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

while(True):
    ret,frame = webcam.read()

    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = at_detector.detect(grey)

    final = frame

    #go through and draw a box around each apriltag it finds
    for det in detections:
        x1 = int(det.corners[0][0])
        y1 = int(det.corners[0][1])

        x2 = int(det.corners[2][0])
        y2 = int(det.corners[2][1])

        final = cv2.rectangle(final,(x1,y1),(x2,y2),(0,255,0),5)

    cv2.imshow('output', final)

    if (cv2.waitKey(40) & 0xFF == ord('q')):
        break

webcam.release()
cv2.destroyAllWindows()
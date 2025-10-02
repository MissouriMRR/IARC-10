import cv2
import numpy as np

filePath = 'PFM1.png'
img = cv2.imread(filePath)

hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

upperBound = np.array([100, 255, 150])
lowerBound = np.array([10, 100, 10])

thresh = cv2.inRange(hsv, lowerBound, upperBound)

cv2.imshow('hsv',hsv)
cv2.imshow('w/ threshold',thresh)
cv2.waitKey(0)
cv2.destroyAllWindows()
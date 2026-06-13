import cv2
import numpy as np

filePath = 'PFM1.png'
img = cv2.imread(filePath)

hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

upperBound = np.array([255, 170, 150])
lowerBound = np.array([20, 110, 110])

thresh = cv2.inRange(hsv, lowerBound, upperBound)

thresh = cv2.medianBlur(thresh,9)

cv2.imshow('hsv',hsv)
cv2.imshow('w/ threshold',thresh)
cv2.waitKey(0)
cv2.destroyAllWindows()
import cv2
import numpy as np

webcam = cv2.VideoCapture(0)

while(True):
    ret,image = webcam.read()
    output = image.copy()

    # -------------------
    # 1. HSV + Contour Filtering
    # -------------------
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Initial HSV range for dull green (tweak by sampling mine pixel with cv2)
    lower_green = np.array([40, 20, 40])
    upper_green = np.array([80, 150, 150])

    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Clean up small details
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        area = cv2.contourArea(c)
        if area > 500:   # adjust threshold
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(output, (x,y), (x+w,y+h), (0,0,255), 2)
            cv2.putText(output, "Candidate", (x,y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
    
    cv2.imshow('output',output)

    if (cv2.waitKey(40) & 0xFF == ord('q')):
        break

webcam.release()
cv2.destroyAllWindows()

#cv2.imwrite("mask_output.png", mask)
#cv2.imwrite("detection_output.png", output)
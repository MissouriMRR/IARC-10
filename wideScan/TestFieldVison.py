import cv2
import numpy as np

# -------------------
# Load image
# -------------------
image = cv2.imread("/home/msz4y/MultirotorSoftware/IARC-10/wideScan/Test_Mine_Pictures/TestPic1.jpg")   # Change path if needed
output = image.copy()

# -------------------
# 1. HSV + Contour Filtering
# -------------------
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Initial HSV range for dull green (tweak by sampling mine pixel with cv2)
lower_green = np.array([150, 200, 200])
upper_green = np.array([210, 255, 255])

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
        print(f'x: {x}\ny: {y}\nw: {w}\nh: {h}')
        cv2.rectangle(output, (x,y), (x+w,y+h), (0,0,255), 2)
        cv2.putText(output, "Candidate", (x,y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

cv2.imwrite("new_mask_output.png", mask)
cv2.imwrite("new_detection_output.png", output)


# -------------------
# 2. Template Matching
# -------------------
# Crop a small template of the mine manually in an image editor OR:
# template = image[y1:y2, x1:x2]   # Replace with actual coords
# For now, load from file if you save it separately
# template = cv2.imread("mine_template.png")

# Example dummy placeholder
# h, w = template.shape[:2]
# result = cv2.matchTemplate(image, template, cv2.TM_CCOEFF_NORMED)
# _, max_val, _, max_loc = cv2.minMaxLoc(result)
# if max_val > 0.5:   # adjust threshold
#     top_left = max_loc
#     bottom_right = (top_left[0] + w, top_left[1] + h)
#     cv2.rectangle(output, top_left, bottom_right, (255,0,0), 2)
#     cv2.putText(output, "Template Match", (top_left[0], top_left[1]-5),
#                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)
# cv2.imshow("Template Matching", output)


cv2.waitKey(0)
cv2.destroyAllWindows()


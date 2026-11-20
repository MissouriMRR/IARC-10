import numpy as np
from PIL import Image
import cv2

image = cv2.imread("test-img1.JPG")

SATURATION = 255
# Shifts hue by hue * 2 degrees (OpenCV values are halved)
HUE_SHIFT = (-162) * 2

# Convert BGR to HSV to change hue & saturation
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

hsv_array = np.array(hsv)

# Changes saturation to max
hsv_array[:, :, 1] = 255

# Changes hue
hue = hsv_array[:, :, 0].astype(np.int16)
hue = (hue + HUE_SHIFT) % 180
hsv_array[:, :, 0] = hue.astype(np.uint8)

result = cv2.cvtColor(hsv_array, cv2.COLOR_HSV2BGR)

cv2.imwrite("output.jpg", result)
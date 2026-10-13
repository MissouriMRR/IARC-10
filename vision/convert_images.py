import numpy as np
from PIL import Image
import cv2

image = cv2.imread("test-img1.JPG").astype(np.float32)

SATURATION = 255
# Shifts hue by hue * 2 degrees (OpenCV values are halved)
HUE_SHIFT = (-162) * 2
TINT = -100
TEMP = -100

# Changes tint
adj1 = TINT / 100 * 50
image[:, :, 0] += adj1
image[:, :, 1] -= adj1
image[:, :, 2] += adj1

# Changes temp
adj2 = np.clip(TEMP, -100, 100) / 100 * 50
image[:,:,0] -= adj2
image[:,:,2] += adj2

image = np.clip(image, 0, 255).astype(np.uint8)

# Convert BGR to HSV to change hue & saturation
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

hsv_array = np.array(hsv)

# Changes hue
hue = hsv_array[:, :, 0].astype(np.int16)
hue = (hue + HUE_SHIFT) % 180
hsv_array[:, :, 0] = hue.astype(np.uint8)




# Changes saturation to max
hsv_array[:, :, 1] = 255

result = cv2.cvtColor(hsv_array, cv2.COLOR_HSV2BGR)

cv2.imwrite("output.jpg", result)
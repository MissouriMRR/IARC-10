import cv2
import numpy as np

# load in the test pictures we have
files = ['wideScan/Test_Mine_Pictures/original_d5f4aee4-ca6a-44cc-9636-234089c94a52_PXL_20251006_231255410.jpg',
         'wideScan/Test_Mine_Pictures/PXL_20251006_231439727.MP.jpg',
         'wideScan/Test_Mine_Pictures/PXL_20251006_231443070.jpg',
         'wideScan/Test_Mine_Pictures/PXL_20251006_231446447.MP.jpg',
         'wideScan/Test_Mine_Pictures/PXL_20251006_231458855.jpg']

for file in files:
    img = cv2.imread(file)
    gray = cv2.cvtColor()
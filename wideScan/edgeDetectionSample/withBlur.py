import cv2
import numpy as np

# load in the test pictures we have
files = ['../Test_Mine_Pictures/original_d5f4aee4-ca6a-44cc-9636-234089c94a52_PXL_20251006_231255410.jpg',
         '../Test_Mine_Pictures/PXL_20251006_231439727.MP.jpg',
         '../Test_Mine_Pictures/PXL_20251006_231443070.jpg',
         '../Test_Mine_Pictures/PXL_20251006_231446447.MP.jpg',
         '../Test_Mine_Pictures/PXL_20251006_231458855.jpg']

# see what we get from each picture
for file in files:
    img = cv2.imread(file)
    #make it gray
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #apply a blur
    blur = cv2.GaussianBlur(gray,(5,5),0)
    #do the edge detection
    edge = cv2.Canny(gray,100,200)

    #thicken the lines
    kernel = np.ones((3, 3), np.uint8)
    thickerLines = cv2.dilate(edge,kernel,iterations=1)

    # try to get rid of some of the extra white space
    #blurAgain = cv2.medianBlur(thickerLines,5)

    #resize the image and show it
    newSize = (int((img.shape[1])*0.3),int((img.shape[0])*0.3))
    resized_image = cv2.resize(thickerLines, newSize, interpolation=cv2.INTER_LINEAR)
    cv2.imshow('final', resized_image)
    #wait to kill the window
    while(True):
        # Check for a key press
        key = cv2.waitKey(1) & 0xFF
        # If 'q' is pressed, break the loop
        if key == ord('q'):  
            break  
    cv2.destroyAllWindows()
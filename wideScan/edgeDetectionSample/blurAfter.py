import cv2
import numpy as np

# load in the test pictures we have
files = ['../Test_Mine_Pictures/original_d5f4aee4-ca6a-44cc-9636-234089c94a52_PXL_20251006_231255410.jpg',
         '../Test_Mine_Pictures/PXL_20251006_231439727.MP.jpg',
         '../Test_Mine_Pictures/PXL_20251006_231443070.jpg',
         '../Test_Mine_Pictures/PXL_20251006_231446447.MP.jpg',
         '../Test_Mine_Pictures/PXL_20251006_231458855.jpg']

for file in files:
    img = cv2.imread(file)
    copy = img
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    edge = cv2.Canny(gray,100,200)

    #thicken the lines
    kernel = np.ones((2, 2), np.uint8)
    thickerLines = cv2.dilate(edge,kernel,iterations=1)

    #median blur after
    blurAfter = cv2.medianBlur(thickerLines,7)

    #          ADD BOUNDING BOXES
    contours, hierarchy = cv2.findContours(blurAfter, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        x,y,w,h = cv2.boundingRect(contour)
        if((w <= 200) and (h <= 200) and ((w*h) > 10000)):
            cv2.rectangle(copy, (x, y), (x + w, y + h), (0, 0, 255), 5)

    ''' From here, I want to only pick the boxes that are close enough to the average value of 
        the different channels some in reference picture(s).
        The only thing is idk what color space to use:
            - hsv might be nice so that we can more easily rule out grass
            - rgb has much more predictable behavior
    '''


    newSize = (int((img.shape[1])*0.3),int((img.shape[0])*0.3))
    resized_image = cv2.resize(copy, newSize, interpolation=cv2.INTER_LINEAR)
    cv2.imshow('with boxes',resized_image)
    while(True):
        # Check for a key press
        key = cv2.waitKey(1) & 0xFF
        # If 'q' is pressed, break the loop
        if key == ord('q'):  
            break  
    cv2.destroyAllWindows()
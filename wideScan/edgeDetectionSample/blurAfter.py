import cv2
import numpy as np
from skimage.metrics import structural_similarity as ssim 
import os

# load in the test pictures we have
files = ['../Test_Mine_Pictures/original_d5f4aee4-ca6a-44cc-9636-234089c94a52_PXL_20251006_231255410.jpg',
         '../Test_Mine_Pictures/PXL_20251006_231439727.MP.jpg',
         '../Test_Mine_Pictures/PXL_20251006_231443070.jpg',
         '../Test_Mine_Pictures/PXL_20251006_231446447.MP.jpg',
         '../Test_Mine_Pictures/PXL_20251006_231458855.jpg']
compareMine = cv2.imread('../IARC_Reference_Mine.png')
compareMine = cv2.cvtColor(compareMine, cv2.COLOR_BGR2GRAY)

for file in files:
    print()
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

    # keep track of how many boxes we're picking up
    num = 0
    #go through all of the contours and see if any of them are mine-like
    for contour in contours:
        x,y,w,h = cv2.boundingRect(contour)
        if((w <= 200) and (h <= 200) and ((w*h) > 10000)):
            mineCropped = gray[x : (x+w), y : (y+h)]
            comparison = cv2.resize(compareMine,(h,w),interpolation=cv2.INTER_LINEAR)
            num += 1
            text = f"Box {num}"
            cv2.putText(copy,text,(x,y), cv2.FONT_HERSHEY_COMPLEX, 2, (0,0,255), 2, cv2.LINE_AA)
            cv2.rectangle(copy, (x, y), (x + w, y + h), (0, 0, 255), 5)
            
            print(f'Box {num}:')
            print(mineCropped.shape)
            print(comparison.shape)
            
            if(mineCropped.shape[0] > 0):
                score,diff = ssim(mineCropped, comparison, full = True)
                print(f"Score for Box {num}: {round((score * 100),2)}")

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
    os.system('clear')
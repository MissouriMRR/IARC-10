import cv2
import numpy as np
from skimage.feature import local_binary_pattern
from scipy.spatial.distance import euclidean
import os

def main():
    # load in the test pictures we have
    files = ['../all_pictures/Test_Mine_Pictures/original_d5f4aee4-ca6a-44cc-9636-234089c94a52_PXL_20251006_231255410.jpg',
            '../all_pictures/Test_Mine_Pictures/PXL_20251006_231439727.MP.jpg',
            '../all_pictures/Test_Mine_Pictures/PXL_20251006_231443070.jpg',
            '../all_pictures/Test_Mine_Pictures/PXL_20251006_231446447.MP.jpg',
            '../all_pictures/Test_Mine_Pictures/PXL_20251006_231458855.jpg']
    ref_pic_path = '../all_pictures/cropped_reference_mine.jpg'
    #get our reference picture and histogram ready
    ref_pic = cv2.imread(ref_pic_path) # picture
    ref_pic = cv2.cvtColor(ref_pic,cv2.COLOR_BGR2GRAY) # needs to be greyscale
    ref_pic_hist = get_lbp_hist(ref_pic) # this is what we'll actually use in our comparisons
    #try all the different pictures we have
    for file in files:
        print()
        # get our image
        img = cv2.imread(file)
        # make some copies that we can draw on
        copy = img
        finalCopy = copy
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # do some edge detection
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
                #show all of the boxes in the normal copy
                num += 1
                text = f"Box {num}"
                cv2.putText(copy,text,(x,y), cv2.FONT_HERSHEY_COMPLEX, 2, (0,0,255), 2, cv2.LINE_AA)
                cv2.rectangle(copy, (x, y), (x + w, y + h), (0, 0, 255), 5)

                print(f'Box{num}: x,y,w,h ={x},{y},{w},{h}')
                #get a cropped image of just the bounding rectangle
                cropped = img
                cropped = cropped[y:(y+h),x:(x+w)]
                if(cropped is None):
                    print('issue with the image')
                else:
                    #make it greyscale
                    cropped = cv2.cvtColor(cropped,cv2.COLOR_BGR2GRAY)
                    cropped = cv2.equalizeHist(cropped)
                    #get the box's lbp histogram
                    cropped_hist = get_lbp_hist(cropped)
                    #get the box's similarity to our sample image
                    comparison = compare_textures(cropped_hist,ref_pic_hist)
                    print(f'Box {num}: {comparison}')
                    print()
        newSize = (int((img.shape[1])*0.3),int((img.shape[0])*0.3))
        copy = cv2.resize(copy, newSize, interpolation=cv2.INTER_LINEAR)
        finalCopy = cv2.resize(finalCopy, newSize, interpolation=cv2.INTER_LINEAR)
        cv2.imshow('All boxes',copy)
        #cv2.imshow('w/ texture matching',finalCopy)
        while(True):
            # Check for a key press
            key = cv2.waitKey(1) & 0xFF
            # If 'q' is pressed, break the loop
            if key == ord('q'):  
                break  
        cv2.destroyAllWindows()
        os.system('clear')

def get_lbp_hist(image):

    #find the lbp first
    lbp = local_binary_pattern(image,8,1,'uniform')

    # this can be tweaked to +2, +1, or +0
    n_bins = 8 + 2

    hist, _ = np.histogram(lbp.ravel(),bins= n_bins, range= (0,n_bins),density= True)

    return hist

def compare_textures(hist1,hist2, method = 'euclidean'):
    if(method == 'euclidean'):
        return euclidean(hist1,hist2)
    # we also might want to try correlation coefficient
    if(method == 'correlation'):
        return np.corrcoef(hist1,hist2)[0,1]

main()
import cv2
import numpy as np
import numpy.typing as npt
from skimage.feature import local_binary_pattern
from scipy.spatial.distance import euclidean
import os
from pathlib import Path

def main():
    # load in the test pictures we have
    '''files: list[str] = ['../all_pictures/Test_Mine_Pictures/original_d5f4aee4-ca6a-44cc-9636-234089c94a52_PXL_20251006_231255410.jpg',
            '../all_pictures/Test_Mine_Pictures/PXL_20251006_231439727.MP.jpg',
            '../all_pictures/Test_Mine_Pictures/PXL_20251006_231443070.jpg',
            '../all_pictures/Test_Mine_Pictures/PXL_20251006_231446447.MP.jpg',
            '../all_pictures/Test_Mine_Pictures/PXL_20251006_231458855.jpg',
            ]'''
    pathToPics = Path('../../../../pictures/2-4-26photos/')
    new_pictures: list[str] = []
    new_pictures.extend(pathToPics.glob("*.jpeg"))
    ref_pic_path: str = '../all_pictures/reference_mine.jpg'
    #get our reference picture and histogram ready
    ref_pic: npt.NDArray[np.uint8] = cv2.imread(ref_pic_path) # picture
    ref_pic = cv2.cvtColor(ref_pic,cv2.COLOR_BGR2GRAY) # needs to be greyscale
    ref_pic_hist: npt.NDArray[np.float64] = get_lbp_hist(ref_pic) # this is what we'll actually use in our comparisons
    #try all the different pictures we have
    file: str
    for file in new_pictures:
        # get our image
        img: npt.NDArray[np.uint8] = cv2.imread(file)
        # make some copies that we can draw on
        copy: npt.NDArray[np.uint8] = img # where I'll draw all boxes that are picked up
        finalCopy: npt.NDArray[np.uint8] = img # where I'll draw the boxes that pass
        gray: npt.NDArray[np.uint8] = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # do some edge detection
        edge: npt.NDArray[np.uint8] = cv2.Canny(gray,100,200)
        #thicken the lines
        kernel: npt.NDArray[np.uint8] = np.ones((2, 2), np.uint8)
        thickerLines: npt.NDArray[np.uint8] = cv2.dilate(edge,kernel,iterations=1)
        #median blur after
        blurAfter: npt.NDArray[np.uint8] = cv2.medianBlur(thickerLines,7)
        #          ADD BOUNDING BOXES
        contours: tuple[npt.NDArray[np.int32]]
        contours , _ = cv2.findContours(blurAfter, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # keep track of how many boxes we're picking up
        num: int = 0
        #go through all of the contours and see if any of them are mine-like
        contour: npt.NDArray[np.int32]
        for contour in contours:
            x: int
            y: int
            w: int
            h: int
            x,y,w,h = cv2.boundingRect(contour)
            if((w <= (0.25 * img.shape[1])) and (h <= (0.25 * img.shape[0])) and ((w*h) > 10000)):
                #show all of the boxes in the normal copy
                num += 1
                text: str = f"Box {num}"
                cv2.putText(copy,text,(x,y), cv2.FONT_HERSHEY_COMPLEX, 3, (255,255,255), 2, cv2.LINE_AA)
                cv2.rectangle(copy, (x, y), (x + w, y + h), (255,255,255), 10)

                '''print(f'Box{num}: x,y,w,h ={x},{y},{w},{h}')'''
                #get a cropped image of just the bounding rectangle
                cropped: npt.NDArray[np.uint8] | None = img
                if(cropped is None):
                    print('issue with the image')
                else:
                    cropped = cropped[y:(y+h),x:(x+w)]
                    #make it greyscale
                    cropped = cv2.cvtColor(cropped,cv2.COLOR_BGR2GRAY)
                    cropped = cv2.equalizeHist(cropped)
                    #get the box's lbp histogram
                    cropped_hist: npt.NDArray[np.float64] = get_lbp_hist(cropped)
                    #get the box's similarity to our sample image
                    comparison: float = compare_textures(cropped_hist,ref_pic_hist,method='euclidean')#bhattacharyya is current best
                    #print(f'Box {num} Confidence Interval: {comparison}')
                    #print()
        # resize copy images so they aren't huge on the screen
        newSize: tuple[int] = (int((img.shape[1])*0.3),int((img.shape[0])*0.3))
        copy = cv2.resize(copy, newSize, interpolation=cv2.INTER_LINEAR)
        finalCopy = cv2.resize(finalCopy, newSize, interpolation=cv2.INTER_LINEAR)
        # show both copies
        cv2.imshow('All boxes',copy)
        #cv2.imshow('w/ texture matching',finalCopy)
        while(True):
            # Check for a key press
            key: int = cv2.waitKey(1) & 0xFF
            # If 'q' is pressed, break the loop
            if key == ord('q'):  
                break  
        cv2.destroyAllWindows()
        os.system('clear')

def get_lbp_hist(image):
    r: int = 2
    p: int = 16*r
    #find the lbp first
    lbp = local_binary_pattern(image,P=p,R=r,method='uniform')

    # this can be tweaked to +2, +1, or +0
    n_bins = 8 + 2

    # the actual function
    hist, _ = np.histogram(lbp.ravel(),bins= n_bins, range= (0,n_bins),density= True)

    return hist

def compare_textures(hist1,hist2, method = 'euclidean'):
    # these are just a bunch of different ways of comparing histograms
    # the best options for our use case are euclidean, bhattacharyya, or intersection
    if(method == 'euclidean'):
        return 100*(euclidean(hist1,hist2))
    if(method == 'correlation'):
        return np.corrcoef(hist1,hist2)[0,1]
    if(method == 'bhattacharyya'):
        return 100*(-np.log(np.sum(np.sqrt(hist1 * hist2)) + 1e-10))
    if(method == 'intersection'):
        return 100*(1 - np.sum(np.minimum(hist1, hist2)))
    if(method == 'chi_squared'):
        return 0.5 * np.sum((hist1 - hist2)**2 / (hist1 + hist2 + 1e-10))
main()
from collections.abc import Iterable

import cv2
import cv2.typing

import numpy as np
import numpy.typing as npt

from vision.common import Attitude, BoundingBox, CameraInfo, Coordinate

from histogram import get_lbp_hist, compare_textures

def detect_mines(
    image: cv2.typing.MatLike,
    coord: Coordinate,
    attitude: Attitude,
    camera_info: CameraInfo,
) -> Iterable[tuple[Coordinate, float]]:
    """
    Detect mines and return coordinates.

    Parameters
    ----------
    coord : Coordinate
        The GPS coordinate of the drone and camera when the image was captured.
    attitude : Attitude
        The combined attitude of the drone and camera when the image was captured.
    camera_info : CameraInfo
        The camera information including field of view.

    Returns
    -------
    Iterable[tuple[Coordinate, float]]
        The coordinates and confidence scores of the detected mines.
    """
    #get all of the boxes from the image detection
    mineBoxes: Iterable[BoundingBox] = detect_mines_in_image(image)
    #returning object that will have the tuples
    coordBoxes: Iterable[tuple[Coordinate, float]]
    #loop through all box images and turn them into coordinates
    for box in mineBoxes:
        coord: Coordinate = Coordinate.from_image_coord(box.x_min, box.y_min, box.width, box.height, attitude, camera_info)
        coordBox: tuple[Coordinate, float] = (coord,box.confidence)
        coordBoxes.append(coordBox)
    return coordBoxes

def detect_mines_in_image(image: cv2.typing.MatLike) -> Iterable[BoundingBox]:
    """
    Detect mines in the given image and return bounding boxes.

    Parameters
    ----------
    image : cv2.typing.MatLike
        The input image in which to detect mines.

    Returns
    -------
    Iterable[BoundingBox]
        The bounding boxes of the detected mines.
    """
    ref_pic_path: str = 'reference_pic.jpg'
    #get our reference picture and histogram ready
    ref_pic: npt.NDArray[np.uint8] = cv2.imread(ref_pic_path) # picture
    ref_pic = cv2.cvtColor(ref_pic,cv2.COLOR_BGR2GRAY) # needs to be greyscale
    ref_pic_hist: npt.NDArray[np.float64] = get_lbp_hist(ref_pic) # this is what we'll actually use in our comparisons
    copy: npt.NDArray[np.uint8] = image # where I'll draw all boxes that are picked up
    finalCopy: npt.NDArray[np.uint8] = image # where I'll draw all boxes that are picked up
    gray: npt.NDArray[np.uint8] = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edge: npt.NDArray[np.uint8] = cv2.Canny(gray,100,200)
    kernel: npt.NDArray[np.uint8] = np.ones((2, 2), np.uint8)
    thickerLines: npt.NDArray[np.uint8] = cv2.dilate(edge,kernel,iterations=1)
    blurAfter: npt.NDArray[np.uint8] = cv2.medianBlur(thickerLines,7)
    contours: tuple[npt.NDArray[np.int32]]
    contours , _ = cv2.findContours(blurAfter, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour: npt.NDArray[np.int32]
    allBoxes: list[BoundingBox]
    confidences: list[float]
    for contour in contours:
        x: int
        y: int
        w: int
        h: int
        x,y,w,h = cv2.boundingRect(contour)
        if((w <= (0.25 * image.shape[1])) and (h <= (0.25 * image.shape[0])) and ((w*h) > 10000)):
            #show all of the boxes in the normal copy
            num += 1
            text: str = f"Box {num}"
            cv2.putText(copy,text,(x,y), cv2.FONT_HERSHEY_COMPLEX, 3, (255,255,255), 2, cv2.LINE_AA)
            cv2.rectangle(copy, (x, y), (x + w, y + h), (255,255,255), 10)

            '''print(f'Box{num}: x,y,w,h ={x},{y},{w},{h}')'''
            #get a cropped image of just the bounding rectangle
            cropped: npt.NDArray[np.uint8] | None = image
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
            box: BoundingBox
            box.x_min = x
            box.y_min = y
            box.width = w
            box.height = h
            box.confidence = comparison
            allBoxes.append(box)
            confidences.append(comparison)
    percentage: int = 10
    threshold: float = np.percentile(confidences,(100-percentage))
    finalBoxes: list[BoundingBox]
    for box in allBoxes:
        if(box.confidence > threshold):
            finalBoxes.append(box)
    return finalBoxes
    raise NotImplementedError("Mine detection not yet implemented.")

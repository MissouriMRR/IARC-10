from ultralytics import YOLO
import cv2
from pathlib import Path


# init the model data structure
pathToModel = "../../models/v11s_2-16.pt"
model = YOLO(pathToModel)

#get a list of all the pictures
pathToPics = Path('../../../pictures/2-12-26_Data_set/train/images/')
allPics: list[str] = []
allPics.extend(pathToPics.glob("*.jpg"))

#detect for every image
for file in allPics:
    results = model(file)
    frame = results[0].plot()

    print(len(results))

    cv2.imshow("window",frame)
    while(True):
        # Check for a key press
        key: int = cv2.waitKey(1) & 0xFF
        # If 'q' is pressed, break the loop
        if key == ord('q'):  
            break
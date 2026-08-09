# Usage
1. Load a config file from somewhere
2. Initialize Camera object with config as the sole parameter
3. Run .start_camera()

## Functions
### capture(save_image, save_detections, save_metadata, drone, force_capture)
`save_image`: bool\
Take a picture and save the result to `pathToPics`\
`save_detections`: bool\
Save detection data to `pathToDetections`\
`save_metadata`: bool\
Save drone metadata to `pathToMetadata`\
`drone`:\
Dronekit object\
`force_capture`: bool\
Force the camera to capture, even if it does not detect anything

## Config
`mainRes`: Camera resolution\
`loresRes`: Low-res camera resolution\
`confThreshold`: Confidence needed to capure (0 - 1)\
`modelPath`: Path to .rpk file\
~~`useLores`: Use low-res stream instead of main~~\
~~`printDetections`: Print detected objects~~\
`pathToPics`: Path to save images to\
`pathToDetections`: Path to save detections to\
`pathToMetadata`: Path to save metadata to\
`droneAddress`: unknown, will ask owen\
`baudRate`: unknown, will ask owen\
`shutterSpeed`: Camera shutter speed in microseconds

---
Public functions are anything that does NOT have an underscore before the function name in camera.py\
Currently this only consists of capture()
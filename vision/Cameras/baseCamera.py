import vision.other.detection
import vision.common.image
import vision.common.detection
import Cameras.Camera_utils.MineDetection



class  BaseCamera():
    def __init__(self):
        pass


    def intialize_camera(self) -> None:
        raise NotImplementedError("intialize_camera method must be implemented by subclasses")

    def capture_and_detect_mines(self) -> 'List[vision.common.detection.Detection]':
        raise NotImplementedError("capture_and_detect_mines method must be implemented by subclasses")
    
    def capture_and_detect_apriltags(self) -> 'List[vision.common.detection.Detection]':
        raise NotImplementedError("capture_and_detect_apriltags method must be implemented by subclasses")

    def capture_image(self,only_metadata: bool) -> 'Image':
        raise NotImplementedError("capture_image method must be implemented by subclasses")


    

    
    
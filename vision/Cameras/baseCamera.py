import Cameras.Camera_utils.detection
import Cameras.Camera_utils.image
import Cameras.Camera_utils.MineDetection
class  BaseCamera():
    def __init__(self):
        pass


    def intialize_camera(self) -> None:
        raise NotImplementedError("intialize_camera method must be implemented by subclasses")

    def capture_and_detect_mines(self) -> 'List[Cameras.Camera_utils.detection.Detection]':
        raise NotImplementedError("capture_and_detect_mines method must be implemented by subclasses")
    

    #RPI cam doesn't need to implement this.
    def capture_image(self) -> 'Image':
        raise NotImplementedError("capture_image method must be implemented by subclasses")

    def detect_mines(self, image: 'Image') -> 'List[Detection]':
        raise NotImplementedError("detect_mines method must be implemented by subclasses")
    
    
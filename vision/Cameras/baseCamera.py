from vision.common.detection import Detection
from vision.common.image import Image



class  BaseCamera():
    def __init__(self):
        pass


    def initialize_camera(self) -> None:
        raise NotImplementedError("intialize_camera method must be implemented by subclasses")

    def capture_and_detect_mines(self) -> list[Detection]:
        raise NotImplementedError("capture_and_detect_mines method must be implemented by subclasses")

    def capture_and_detect_apriltags(self) -> list[Detection]:
        raise NotImplementedError("capture_and_detect_apriltags method must be implemented by subclasses")

    def capture_image(self, _only_metadata: bool) -> Image:
        raise NotImplementedError("capture_image method must be implemented by subclasses")


    

    
    
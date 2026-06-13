#Pretty much only holds the list of detections found in an image.

#BOX STORES THE center x, center y, width and height.
class Detection:
    def __init__(self, score, box: tuple[int, int, int, int], imageSize: tuple[int, int]):
        self.score = score
        self.box = box
        self.imageSize = imageSize

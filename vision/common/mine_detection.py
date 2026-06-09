#Pretty much only holds the list of mines found in an image.
class MineDetection:
    def __init__(self, score, box: tuple[int, int, int, int], imageSize: tuple[int, int]):
        self.score = score
        self.box = box
        self.imageSize = imageSize

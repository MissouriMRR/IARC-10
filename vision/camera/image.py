class Image():
    def __init__(self, mine_list: tuple[MineDetection], corner_coords: Tuple[int, int, int, int]):
        self.mine_list = mine_list 
        self.corner_coords = corner_coords # effectively the dimensions of the image, (0, 0, x, y)

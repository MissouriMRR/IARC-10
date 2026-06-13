class Mine:
    def __init__(self, confidence: float, world_coords: tuple[float, float]):
        self.confidence = confidence
        self.world_coords = world_coords
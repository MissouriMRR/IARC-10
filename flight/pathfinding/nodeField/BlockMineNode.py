from flight.pathfinding.nodeField.node import Node

class BlockMineNode(Node):
    def __init__(self,x,y,direction):
        super().__init__(x,y,False)
        self.direction=direction
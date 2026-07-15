

        

class pathfinding():
    def __init__(self, cornerCoordinates):
        self.gridField=BlockField()
        fieldWidth=cornerCoordinates[1][0]-cornerCoordinates[0][0]
        fieldHeight=cornerCoordinates[2][1]-cornerCoordinates[1][1]
        blocksWidth=fieldWidth//SQUARE_SIDE_LENGTH_FT
        blocksHeight=fieldHeight//SQUARE_SIDE_LENGTH_FT

        BlockField.NUM_COLS=blocksWidth
        BlockField.NUM_ROWS=blocksHeight

        

        
        self.nodeField=Field((cornerCoordinates[2][0],cornerCoordinates[2][1]),cornerCoordinates)

    


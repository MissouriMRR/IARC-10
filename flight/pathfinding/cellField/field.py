class cell:
    #variables
    __mine:bool = False
    __neighborMine = False
    __nextInPath:cell = None
    __neighbors:list[cell] = None
    __y:int = None
    __x:int = None

    #functions
    def __init__(self, x:int, y:int):
        self.__y = y
        self.__x = x

    def impassible(self) -> bool:
        return not (self.__mine or self.__neighborMine)
    
    def addNeighbor(self, neighbor) -> None:
        if (self.__neighbors is None):
            self.__neighbors = [neighbor]
        else:
            self.__neighbors.append(neighbor)

    def setNeighborMine(self) -> None:
        self.__neighborMine = True

    def addMine(self) -> None:
        self.__mine = True
        for neighbor in self.__neighbors:
            neighbor.setNeighborMine()

    def setNext(self, other:cell) -> None:
        self.__nextInPath = other


    def __str__(self):
        if self.__mine:
            return 'X'
        if self.__neighborMine:
            return 'x'
        if (self.__nextInPath):
            next = self.__nextInPath
            switch = (next.__x-self.__x)*2 + (next.__y-self.__y)
            match switch:
                case 1:
                    return '▼'
                case -1:
                    return '▲'
                case 2:
                    return '►'
                case -2:
                    return '◄'
        return ' '
    

            



class field:
    #variables
    __Field: list[list[cell]] = None
    __width: int = None
    __height: int = None

    #functions
    def __init__(self, width:int, height:int) -> None:
        self.__width = width
        self.__height = height
        self.__Field = [[cell(x,y) for x in range(width)] for y in range(height)]
        for x in range(width):
            for y in range(height):
                currentCell = self.__Field[y][x]
                for dir in ((0,1),(0,-1),(-1,0),(1,0)):
                    tx = x+dir[0]
                    ty = y+dir[1]
                    if (tx<0 or tx>=width):
                        continue
                    if (ty<0 or ty>=height):
                        continue
                    currentCell.addNeighbor(self.__Field[ty][tx])
    
    def impassible(self, x:int, y:int) -> bool:
        return self.__Field[y][x].impassible()
    
    def addMine(self, x:int, y:int) -> None:
        self.__Field[y][x].addMine()

    #Need to add safety checks for ensuring no mines along side valid movemnt
    def addPathSegment(self, x1:int, y1:int, x2:int, y2:int) -> None: 
        self.__Field[y1][x1].setNext(self.__Field[y2][x2])

    def __str__(self):
        out = []
        out.append('-'*(self.__width+2))
        for row in self.__Field:
            out.append(f"|{''.join(str(c) for c in row)}|")
        out.append('-'*(self.__width+2))
        return '\n'.join(out)
    
if (__name__ == "__main__"):
    grid = field(10, 10)
    grid.addMine(5,5)
    grid.addMine(8,3)
    grid.addPathSegment(0,0,0,1)
    grid.addPathSegment(0,1,1,1)
    grid.addPathSegment(1,1,1,2)
    grid.addPathSegment(1,2,1,3)
    print(grid)
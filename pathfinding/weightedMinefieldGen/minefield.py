import random
from copy import deepcopy

class Minefield:
    pathSymbol = "O"
    mineSymbol = "M"
    dangerZoneSymbol = "X"
    emptySymbol= " "

    def __init__(self,length:int,width:int,randomized=True):
        self.length = length
        self.width = width
        self.minefield = [[Minefield.emptySymbol for i in range(width)] for i in range(length)]
        if not randomized:
            random.seed(136)

    # Paths will always favor going up, will change if that is not the case
    def createPath(self,startPoint:list):
        direction = random.randint(1,100)
        leftWeight = [i for i in range(1,26)] # 1-25
        rightWeight = [i for i in range(26,51)] # 26-50
        upWeight = [i for i in range(51,80)]# 51-75
        downWeight = [i for i in range(81,101)]# 76-100
        
        referencePoint = deepcopy(startPoint) # [Row, Col] or [y,x], [0,0] is at the top left of grid
        mines = [Minefield.mineSymbol, Minefield.dangerZoneSymbol]
        while referencePoint[0] > 0:
            direction = random.randint(1,100)
            if direction in leftWeight and (referencePoint[1] - 1 >= 0):
                if self.minefield[referencePoint[0]][referencePoint[1]-1] not in mines:
                    referencePoint[1] -= 1 # Left
                else:
                    continue
            elif direction in rightWeight and (referencePoint[1] + 1 < self.width):
                if self.minefield[referencePoint[0]][referencePoint[1] + 1] not in mines:
                    referencePoint[1] += 1 # Right
                else:
                    continue
            elif direction in upWeight and (referencePoint[0] - 1 >= 0):
                if self.minefield[referencePoint[0] - 1][referencePoint[1]] not in mines:
                    referencePoint[0] -= 1 # Up
                else:
                    continue
            elif direction in downWeight and (referencePoint[0] + 1 < self.length):
                if self.minefield[referencePoint[0] + 1][referencePoint[1]] not in mines:
                    referencePoint[0] += 1 # Down
                else:
                    continue

            self.minefield[referencePoint[0]][referencePoint[1]] = Minefield.pathSymbol

    # Mines Generate with a set radius, numOfMines must be between 0 - Field length or width.
    # Going significantly over will result in filling the field with as much mines possible 
    # around paths.
    def generateMines(self,numOfMines:int,radius:int=1):
        placeable = True
        for i in range(numOfMines):
            referencePoint = [random.randint(0,self.length-1),random.randint(0,self.width-1)]
            if self.minefield[referencePoint[0]][referencePoint[1]] not in [Minefield.pathSymbol,Minefield.mineSymbol,Minefield.dangerZoneSymbol]:
                placeable = True
            else:
                placeable = False
                continue
            
            if placeable:
                self.minefield[referencePoint[0]][referencePoint[1]] = Minefield.mineSymbol
            else:
                continue

            for k in range(1,radius+1): # Checking if radius is not occupied.
                leftShift = k           # Also adding radius symbols
                rightShift = k
                upShift = k
                downShift = k

                # Prevents radius from overflowing to negative or index out of range
                if (referencePoint[1]-leftShift < 0 ): 
                    leftShift = 0
                if (referencePoint[1]+rightShift > self.width-1):
                    rightShift = 0
                if (referencePoint[0]-upShift < 0):
                    upShift = 0
                if (referencePoint[0]+downShift > self.length-1):
                    downShift = 0
                
                nonos = [Minefield.pathSymbol,Minefield.mineSymbol,Minefield.dangerZoneSymbol]
                
                ### Check each radius cell if it is occupied ###
                if placeable:
                    for l in range(leftShift+1): # Fill the upper left quadrant
                        for d in range(upShift+1):
                            if self.minefield[referencePoint[0]-d][referencePoint[1]-l] not in nonos:
                                    self.minefield[referencePoint[0]-d][referencePoint[1]-l] = Minefield.dangerZoneSymbol
                            else:
                                continue
                    
                    for l in range(rightShift+1): # Fill the upper Right quadrant
                        for d in range(upShift+1):
                            if self.minefield[referencePoint[0]-d][referencePoint[1]+l] not in nonos:
                                    self.minefield[referencePoint[0]-d][referencePoint[1]+l] = Minefield.dangerZoneSymbol
                            else:
                                continue
                    
                    for l in range(leftShift+1): # Fill the lower left quadrant
                        for d in range(downShift+1):
                            if self.minefield[referencePoint[0]+d][referencePoint[1]-l] not in nonos:
                                    self.minefield[referencePoint[0]+d][referencePoint[1]-l] = Minefield.dangerZoneSymbol
                            else:
                                continue
                    
                    for l in range(rightShift+1): # Fill the lower Right quadrant
                        for d in range(downShift+1):
                            if self.minefield[referencePoint[0]+d][referencePoint[1]+l] not in nonos:
                                    self.minefield[referencePoint[0]+d][referencePoint[1]+l] = Minefield.dangerZoneSymbol
                            else:
                                continue

                if not placeable:
                    break
    
    # Displaying Mines/Paths only isolates their corresponding values when visualizing
    def displayOnlyMines(self):
        print("  -  "*self.length)
        print("Displaying Mines Only")

        mineOnlyField = deepcopy(self.minefield)
        for r,row in enumerate(mineOnlyField):
            for c,col in enumerate(row):
                if mineOnlyField[r][c] in [Minefield.pathSymbol]:
                    mineOnlyField[r][c] = Minefield.emptySymbol
        for value in mineOnlyField:
            print() 
            print(value)
        
        print("Displaying Mines Only")
        print("  -  "*self.length)
        return mineOnlyField

    def displayOnlyPaths(self):
        print("  -  "*self.length)
        print("Displaying Paths Only")
        pathsOnlyField = deepcopy(self.minefield)
        for r,row in enumerate(pathsOnlyField):
            for c,col in enumerate(row):
                if pathsOnlyField[r][c] in [Minefield.mineSymbol,Minefield.dangerZoneSymbol]:
                    pathsOnlyField[r][c] = Minefield.emptySymbol
        for value in pathsOnlyField:
            print() 
            print(value)
        
        print("Displaying Paths only")
        print("  -  "*self.length)
        return pathsOnlyField
    
    def get(self):
        return self.minefield

    def __str__(self):
        for value in self.minefield:
            print()
            print(value)
        return ' '.join(self.minefield)

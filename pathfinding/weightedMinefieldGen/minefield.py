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
    # Paths are made under the assumption that no mines have been placed yet
    def createPath(self,startPoint:list,reactive:bool = None,fillDiagonals:bool = True):
        
        if reactive == None:
            if random.randint(0,1) == 0:
                reactive = True
            else:
                reactive = False
        referencePoint = startPoint # [Row, Col]
        self.minefield[referencePoint[0]][referencePoint[1]] = Minefield.pathSymbol

        while referencePoint[0] > 0:
            # Reactive - Changes direction weights to favor a certain direction
            if reactive:
                if referencePoint[1] <= self.width//4:
                    directionRange = [1,15]
                    leftWeight = [1,11]
                    upLeftWeight = [2,12]
                    upWeight = [3,13]
                    upRightWeight = [4,9,14]
                    rightWeight = [5,10,15]
                    downLeftWeight = [6]
                    downWeight = [7]
                    downRightWeight = [8]
                elif referencePoint[1] >= 2*(self.width//4):
                    directionRange = [1,15]
                    leftWeight = [1,9,11]
                    upLeftWeight = [2,10,12]
                    upWeight = [3,13]
                    upRightWeight = [4,14]
                    rightWeight = [5,15]
                    downLeftWeight = [6]
                    downWeight = [7]
                    downRightWeight = [8]
                else:
                    directionRange = [1,12]
                    leftWeight = [1,9]
                    upLeftWeight = [2,10]
                    upWeight = [3,11]
                    upRightWeight = [4,12]
                    rightWeight = [5,13]
                    downLeftWeight = [6]
                    downWeight = [7]
                    downRightWeight = [8]
            else:
                    directionRange = [1,12]
                    leftWeight = [1,9]
                    upLeftWeight = [2,10]
                    upWeight = [3,11]
                    upRightWeight = [4,12]
                    rightWeight = [5,13]
                    downLeftWeight = [6]
                    downWeight = [7]
                    downRightWeight = [8]
                
            direction = random.randint(directionRange[0],directionRange[1])
            currentRef = deepcopy(referencePoint)

            # Leftward Movement
            if referencePoint[0] >= 1:
                if direction in leftWeight: # Left
                    if referencePoint[1] >= 1:
                        referencePoint[1] -= 1
                elif direction in upLeftWeight: # Upper Left
                    if referencePoint[1] >= 1:
                            referencePoint[0] -= 1
                            referencePoint[1] -= 1
                elif direction in downLeftWeight: # Down Left
                    if referencePoint[0] < self.length-1:
                        if referencePoint[1] >= 1:
                            referencePoint[0] += 1
                            referencePoint[1] -= 1

            #Upward Movement
            if direction in upWeight: # Up
                referencePoint[0] -= 1
            # Downward Movement
            if referencePoint[0] < self.length-1:
                if direction in downWeight: # Down
                    referencePoint[0] += 1
            
            # Rightward Movement
            if referencePoint[0] <= self.length-1 and referencePoint[1] < self.width-1:
                if direction in upRightWeight: # Upper Right
                    if referencePoint[1] < self.length-1:
                        referencePoint[0] -= 1
                        referencePoint[1] += 1
                elif direction in rightWeight: # Right
                    if referencePoint[1] < self.length-1:
                        referencePoint[1] += 1
                elif direction in downRightWeight: # Down Right
                    if referencePoint[0] < self.length-1:
                        if referencePoint[1] <= self.width-1:
                            referencePoint[0] += 1
                            referencePoint[1] += 1 

            # Final Check to see if the referencePoint isnt already marked pathSymbol
            if self.minefield[referencePoint[0]][referencePoint[1]] != Minefield.pathSymbol:
                self.minefield[referencePoint[0]][referencePoint[1]] = Minefield.pathSymbol
                referencePoint = currentRef
                continue
            else:
                self.minefield[referencePoint[0]][referencePoint[1]] = Minefield.pathSymbol

            
            # Check and add borders for smoother paths
            # if fillDiagonals:
            #     leftOffset = 1
            #     upOffset = 1
            #     rightOffset = 1
            #     downOffset = 1

            #     if referencePoint[1] - leftOffset < 0:
            #         leftOffset = 0
            #     if referencePoint[1] + rightOffset > self.width-1:
            #         rightOffset = 0
            #     if referencePoint[0] - upOffset < 0:
            #         upOffset = 0
            #     if referencePoint[0] + downOffset > self.length-1:
            #         downOffset = 0
            #     if random.randint(0,1)==1:
            #         if self.minefield[referencePoint[0]][referencePoint[1]-leftOffset] != Minefield.pathSymbol: # Direct left of referencePoint
            #             self.minefield[referencePoint[0]][referencePoint[1]-leftOffset] = Minefield.pathSymbol
            #         if self.minefield[referencePoint[0]-upOffset][referencePoint[1]-leftOffset] != Minefield.pathSymbol: # Up left of referencePoint
            #             self.minefield[referencePoint[0]-upOffset][referencePoint[1]-leftOffset] = Minefield.pathSymbol
            #         if self.minefield[referencePoint[0]+downOffset][referencePoint[1]-leftOffset] != Minefield.pathSymbol: # Down Left of referencePoint
            #             self.minefield[referencePoint[0]+downOffset][referencePoint[1]-leftOffset] = Minefield.pathSymbol            
            #     else:
            #         if self.minefield[referencePoint[0]][referencePoint[1]+rightOffset] != Minefield.pathSymbol: # Direct right of referencePoint
            #             self.minefield[referencePoint[0]][referencePoint[1]+rightOffset] = Minefield.pathSymbol
            #         if self.minefield[referencePoint[0]-upOffset][referencePoint[1]+rightOffset] != Minefield.pathSymbol: # Up right of referencePoint
            #             self.minefield[referencePoint[0]-upOffset][referencePoint[1]+rightOffset] = Minefield.pathSymbol
            #         if self.minefield[referencePoint[0]+downOffset][referencePoint[1]+rightOffset] != Minefield.pathSymbol: # Down Right of referencePoint
            #             self.minefield[referencePoint[0]+downOffset][referencePoint[1]+rightOffset] = Minefield.pathSymbol            

            #     if self.minefield[referencePoint[0]-upOffset][referencePoint[1]] != Minefield.pathSymbol: # Directly up of referencePoint
            #         self.minefield[referencePoint[0]-upOffset][referencePoint[1]] = Minefield.pathSymbol
                
            #     if self.minefield[referencePoint[0]+downOffset][referencePoint[1]] != Minefield.pathSymbol: # Directly down of referencePoint
            #         self.minefield[referencePoint[0]+downOffset][referencePoint[1]] = Minefield.pathSymbol

            

            


    
    def generateMines(self,numOfMines:int,radius:int=1):
        
        placeable = True

        for i in range(numOfMines):
            referencePoint = [random.randint(0,self.length-1),random.randint(0,self.width-1)]

            if self.minefield[referencePoint[0]][referencePoint[1]] not in [Minefield.pathSymbol,Minefield.mineSymbol,Minefield.dangerZoneSymbol]:
                placeable = True
            else:
                placeable = False
            
            if placeable:
                self.minefield[referencePoint[0]][referencePoint[1]] = Minefield.mineSymbol
            else:
                continue

            for k in range(1,radius+1): # Checking if radius is not occupied, also adding radius symbols
                leftShift = k
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
                # Check each cell to the direct left of referencePoint
                if self.minefield[referencePoint[0]][referencePoint[1]-leftShift] not in nonos: 
                    if placeable:
                        self.minefield[referencePoint[0]][referencePoint[1]-leftShift] = Minefield.dangerZoneSymbol
                else:
                    placeable = False
                # Check each cell to the direct right of referencePoint
                if self.minefield[referencePoint[0]][referencePoint[1]+rightShift] not in nonos: 
                    if placeable:
                        self.minefield[referencePoint[0]][referencePoint[1]+rightShift] = Minefield.dangerZoneSymbol
                else:
                    placeable = False
                
                # Check each cell directly above referencePoint
                if self.minefield[referencePoint[0]-upShift][referencePoint[1]] not in nonos:
                    if placeable:
                        self.minefield[referencePoint[0]-upShift][referencePoint[1]] = Minefield.dangerZoneSymbol
                else:
                    placeable = False
                # Check each cell directly below referencePoint
                if self.minefield[referencePoint[0]+downShift][referencePoint[1]] not in nonos: 
                    if placeable:
                        self.minefield[referencePoint[0]+downShift][referencePoint[1]] = Minefield.dangerZoneSymbol
                else:
                    placeable = False
                
                # Check each cell to the top left of referencePoint
                if self.minefield[referencePoint[0]-upShift][referencePoint[1]-leftShift] not in nonos:
                    if placeable:
                        self.minefield[referencePoint[0]-upShift][referencePoint[1]-leftShift] = Minefield.dangerZoneSymbol
                else:
                    placeable = False
                # Check each cell to the top right of referencePoint
                if self.minefield[referencePoint[0]-upShift][referencePoint[1]+rightShift] not in nonos:
                    if placeable:
                        self.minefield[referencePoint[0]-upShift][referencePoint[1]+rightShift] = Minefield.dangerZoneSymbol
                else:
                    placeable = False
                # Check each cell to the bottom left of referencePoint
                if self.minefield[referencePoint[0]+downShift][referencePoint[1]-leftShift] not in nonos:
                    if placeable:
                        self.minefield[referencePoint[0]+downShift][referencePoint[1]-leftShift] = Minefield.dangerZoneSymbol
                else:
                    placeable = False
                # Check each cell to the bottom right of referencePoint
                if self.minefield[referencePoint[0]+downShift][referencePoint[1]+rightShift] not in nonos:
                    if placeable:
                        self.minefield[referencePoint[0]+downShift][referencePoint[1]+rightShift] = Minefield.dangerZoneSymbol
                else:
                    placeable = False
                
                if not placeable:
                    break
            
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
    

    def get(self):
        return self.minefield

    def __str__(self):
        for value in self.minefield:
            print()
            print(value)
        return "  -  "*self.length

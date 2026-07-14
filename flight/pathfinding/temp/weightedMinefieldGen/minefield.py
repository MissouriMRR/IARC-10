import random
from copy import deepcopy


class Minefield:
    pathSymbol = "O"
    mineSymbol = "M"
    falseMineSymbol = "N"
    dangerZoneSymbol = "X"
    otherObstaclesSymbol = "E"
    emptySymbol = " "

    def __init__(self, length: int, width: int, randomized=True):
        self.length = length
        self.width = width
        self.minefield = [[Minefield.emptySymbol for i in range(width)] for i in range(length)]
        if not randomized:
            random.seed(136)

    # Paths will always favor going up, will change if that is not the case
    def createPath(self, startPoint: list):
        direction = random.randint(1, 100)
        leftWeight = [i for i in range(1, 26)]  # 1-25
        rightWeight = [i for i in range(26, 51)]  # 26-50
        upWeight = [i for i in range(51, 80)]  # 51-75
        downWeight = [i for i in range(81, 101)]  # 76-100

        referencePoint = deepcopy(
            startPoint
        )  # [Row, Col] or [y,x], [0,0] is at the top left of grid
        mines = [Minefield.mineSymbol, Minefield.dangerZoneSymbol]
        while referencePoint[0] > 0:
            direction = random.randint(1, 100)
            if direction in leftWeight and (referencePoint[1] - 1 >= 0):
                if self.minefield[referencePoint[0]][referencePoint[1] - 1] not in mines:
                    referencePoint[1] -= 1  # Left
                else:
                    continue
            elif direction in rightWeight and (referencePoint[1] + 1 < self.width):
                if self.minefield[referencePoint[0]][referencePoint[1] + 1] not in mines:
                    referencePoint[1] += 1  # Right
                else:
                    continue
            elif direction in upWeight and (referencePoint[0] - 1 >= 0):
                if self.minefield[referencePoint[0] - 1][referencePoint[1]] not in mines:
                    referencePoint[0] -= 1  # Up
                else:
                    continue
            elif direction in downWeight and (referencePoint[0] + 1 < self.length):
                if self.minefield[referencePoint[0] + 1][referencePoint[1]] not in mines:
                    referencePoint[0] += 1  # Down
                else:
                    continue

            self.minefield[referencePoint[0]][referencePoint[1]] = Minefield.pathSymbol

    # Mines Generate with a set radius, numOfMines must be between 0 - Field length or width.
    # Going significantly over will result in filling the field with as much mines possible
    # around paths.
    # False mines might be placed, its a bug that I will just embrace into a feature
    def generateMines(self, numOfMines: int, radius: int = 1):
        placeable = True
        for i in range(numOfMines):
            referencePoint = [random.randint(0, self.length - 1), random.randint(0, self.width - 1)]
            if self.minefield[referencePoint[0]][referencePoint[1]] not in [
                Minefield.pathSymbol,
                Minefield.mineSymbol,
                Minefield.dangerZoneSymbol,
            ]:
                placeable = True
            else:
                placeable = False
                continue

            if placeable:
                self.minefield[referencePoint[0]][referencePoint[1]] = Minefield.mineSymbol
            else:
                continue

            for k in range(1, radius + 1):  # Checking if radius is not occupied.
                leftShift = k  # Also adding radius symbols
                rightShift = k
                upShift = k
                downShift = k

                # Prevents radius from overflowing to negative or index out of range
                if referencePoint[1] - leftShift < 0:
                    leftShift = 0
                if referencePoint[1] + rightShift > self.width - 1:
                    rightShift = 0
                if referencePoint[0] - upShift < 0:
                    upShift = 0
                if referencePoint[0] + downShift > self.length - 1:
                    downShift = 0

                nonos = [Minefield.pathSymbol, Minefield.mineSymbol, Minefield.dangerZoneSymbol]

                ### Check each radius cell if it is occupied ###
                if placeable:
                    for l in range(leftShift + 1):  # Fill the upper left quadrant
                        for d in range(upShift + 1):
                            if (
                                self.minefield[referencePoint[0] - d][referencePoint[1] - l]
                                not in nonos
                            ):
                                self.minefield[referencePoint[0] - d][
                                    referencePoint[1] - l
                                ] = Minefield.dangerZoneSymbol
                            else:
                                continue

                    for l in range(rightShift + 1):  # Fill the upper Right quadrant
                        for d in range(upShift + 1):
                            if (
                                self.minefield[referencePoint[0] - d][referencePoint[1] + l]
                                not in nonos
                            ):
                                self.minefield[referencePoint[0] - d][
                                    referencePoint[1] + l
                                ] = Minefield.dangerZoneSymbol
                            else:
                                continue

                    for l in range(leftShift + 1):  # Fill the lower left quadrant
                        for d in range(downShift + 1):
                            if (
                                self.minefield[referencePoint[0] + d][referencePoint[1] - l]
                                not in nonos
                            ):
                                self.minefield[referencePoint[0] + d][
                                    referencePoint[1] - l
                                ] = Minefield.dangerZoneSymbol
                            else:
                                continue

                    for l in range(rightShift + 1):  # Fill the lower Right quadrant
                        for d in range(downShift + 1):
                            if (
                                self.minefield[referencePoint[0] + d][referencePoint[1] + l]
                                not in nonos
                            ):
                                self.minefield[referencePoint[0] + d][
                                    referencePoint[1] + l
                                ] = Minefield.dangerZoneSymbol
                            else:
                                continue

                if not placeable:
                    break

        # A Bug turned feature, mines detected to not fit the following criteria will be marked as a False Mine:
        # - There is another mine adjacent or within the radius of the current mine
        # - There is a safe path adjacent or within the radius of the current mine
        falseMineCriteria = [Minefield.pathSymbol, Minefield.mineSymbol]
        for row in range(len(self.minefield)):
            for col in range(len(self.minefield[0])):
                falseMine = False  # If True, mark the mine as False
                if self.minefield[row][col] in [Minefield.mineSymbol]:
                    for r in range(1, radius + 1):
                        leftShift = r
                        rightShift = r
                        upShift = r
                        downShift = r
                        if row + downShift > len(self.minefield) - 1:  # Prevents overflow
                            downShift = 0
                        else:
                            downShift = r
                        if row - upShift < -1:
                            upShift = 0
                        else:
                            upShift = r
                        # If either direction contains the falseMineCriteria, the current mine becomes
                        # false and cannot be overriden within the main loop over the minefield.
                        if (
                            self.minefield[row + downShift][col] in falseMineCriteria
                            or self.minefield[row - upShift][col] in falseMineCriteria
                        ):
                            falseMine = True

                        if col + rightShift > len(self.minefield[0]) - 1:  # Prevents overflow
                            rightShift = 0
                        else:
                            rightShift = r
                        if row - leftShift <= 0:
                            leftShift = 0
                        else:
                            leftShift = r
                        # If either direction contains the falseMineCriteria, the current mine becomes
                        # false and cannot be overriden within the main loop over the minefield.
                        if (
                            self.minefield[row][col + rightShift] in falseMineCriteria
                            or self.minefield[row][col - leftShift] in falseMineCriteria
                        ):
                            falseMine = True

                    if falseMine:
                        self.minefield[row][col] = Minefield.falseMineSymbol

    # Run AFTER paths and mines have been generated
    def generateOtherObstacles(self, numOfObstacles: int):
        minefield = self.minefield
        shapes = {1: "square", 2: "rectangle", 3: "wall"}  # Or line
        orientation = {
            1: "horizontalLeft",
            2: "horizontalRight",
            3: "verticalUp",
            4: "verticalDown",
        }
        shapeWidth = 0
        shapeLength = 0
        startPoint = [0, 0]  # [Row][Col]
        selectedShape = 1
        selectedOrientation = 1

        for i in range(numOfObstacles):
            startPoint[0] = random.randint(0, self.width - 1)
            startPoint[1] = random.randint(0, self.length - 1)
            selectedShape = random.randint(1, 1)
            selectedOrientation = random.randint(1, 4)

            if shapes[selectedShape] == "square":  # Make a square obstacle
                shapeWidth = random.randint(2, int(round(self.width / 12)))
                if orientation[selectedOrientation] == "horizontalLeft":
                    for width in range(shapeWidth + 1):
                        for length in range(shapeWidth + 1):
                            if random.randint(0, 1) == 0:  # Up
                                if (
                                    startPoint[0] - length > 0 and startPoint[1] - width > 0
                                ) and minefield[startPoint[0] - length][
                                    startPoint[1] - width
                                ] not in [
                                    Minefield.pathSymbol,
                                    Minefield.otherObstaclesSymbol,
                                ]:
                                    minefield[startPoint[0] - length][
                                        startPoint[1] - width
                                    ] == Minefield.otherObstaclesSymbol
                            else:  # Down
                                if (
                                    startPoint[0] + length > 0 and startPoint[1] - width > 0
                                ) and minefield[startPoint[0] + length][
                                    startPoint[1] - width
                                ] not in [
                                    Minefield.pathSymbol,
                                    Minefield.otherObstaclesSymbol,
                                ]:
                                    minefield[startPoint[0] + length][
                                        startPoint[1] - width
                                    ] == Minefield.otherObstaclesSymbol
                elif orientation[selectedOrientation] == "horizontalRight":
                    for width in range(shapeWidth + 1):
                        for length in range(shapeWidth + 1):
                            if random.randint(0, 1) == 0:  #  Up
                                if (
                                    startPoint[0] - length > 0
                                    and startPoint[1] - width < self.width
                                ) and minefield[startPoint[0] - length][
                                    startPoint[1] + width
                                ] not in [
                                    Minefield.pathSymbol,
                                    Minefield.otherObstaclesSymbol,
                                ]:
                                    minefield[startPoint[0] - length][
                                        startPoint[1] + width
                                    ] == Minefield.otherObstaclesSymbol
                            else:  # Down
                                if (
                                    startPoint[0] - length > 0
                                    and startPoint[1] + width < self.width
                                ) and minefield[startPoint[0] + length][
                                    startPoint[1] + width
                                ] not in [
                                    Minefield.pathSymbol,
                                    Minefield.otherObstaclesSymbol,
                                ]:
                                    minefield[startPoint[0] + length][
                                        startPoint[1] + width
                                    ] == Minefield.otherObstaclesSymbol
                elif orientation[selectedOrientation] == "verticalUp":
                    for width in range(shapeWidth + 1):
                        for length in range(shapeWidth + 1):
                            if startPoint[1] - width // 2 > 0:
                                startPoint[1] = -width // 2
                            else:
                                break
                            if random.randint(0, 1) == 0:  # Up
                                if (
                                    startPoint[0] - length > 0 and startPoint[1] - width > 0
                                ) and minefield[startPoint[0] - length][
                                    startPoint[1] - width
                                ] not in [
                                    Minefield.pathSymbol,
                                    Minefield.otherObstaclesSymbol,
                                ]:
                                    minefield[startPoint[0] - length][
                                        startPoint[1] - width
                                    ] == Minefield.otherObstaclesSymbol
                elif orientation[selectedOrientation] == "verticalDown":
                    for width in range(shapeWidth + 1):
                        for length in range(shapeWidth + 1):
                            if startPoint[1] - width // 2 > 0:
                                startPoint[1] = -width // 2
                            else:
                                # Down
                                if (
                                    startPoint[0] + length > 0 and startPoint[1] - width > 0
                                ) and minefield[startPoint[0] + length][
                                    startPoint[1] - width
                                ] not in [
                                    Minefield.pathSymbol,
                                    Minefield.otherObstaclesSymbol,
                                ]:
                                    minefield[startPoint[0] + length][
                                        startPoint[1] - width
                                    ] == Minefield.otherObstaclesSymbol
            elif shapes[selectedShape] == "rectangle":  # Make a Rectangular obstacle
                shapeWidth = random.randint(2, int(round(self.width / 12)))
                shapeLength = random.randint(2, int(round(self.length / 12)))
                if orientation[selectedOrientation] == "horizontalLeft":
                    pass
                elif orientation[selectedOrientation] == "horizontalRight":
                    pass
                elif orientation[selectedOrientation] == "verticalUp":
                    pass
                elif orientation[selectedOrientation] == "verticalDown":
                    pass
            elif shapes[selectedShape] == "wall":  # Make a wall obstacle
                shapeWidth = random.randint(2, int(round(self.width / 12)))
                if orientation[selectedOrientation] == "horizontalLeft":
                    pass
                elif orientation[selectedOrientation] == "horizontalRight":
                    pass
                elif orientation[selectedOrientation] == "verticalUp":
                    pass
                elif orientation[selectedOrientation] == "verticalDown":
                    pass

    # Displaying FalseMines/Mines/Paths only isolates their corresponding values when printing
    def displayOnlyMines(self, print: bool = True):

        mineOnlyField = deepcopy(self.minefield)
        for r, row in enumerate(mineOnlyField):
            for c, col in enumerate(row):
                if mineOnlyField[r][c] in [Minefield.pathSymbol, Minefield.falseMineSymbol]:
                    mineOnlyField[r][c] = Minefield.emptySymbol
        if print:
            print("  -  " * self.length)
            print("Displaying Mines Only")
            for value in mineOnlyField:
                print()
                print(value)
            print("Displaying Mines Only")
            print("  -  " * self.length)
        return mineOnlyField

    def displayOnlyFalseMines(self, print: bool = True):
        falseOnlyField = deepcopy(self.minefield)
        for r, row in enumerate(falseOnlyField):
            for c, col in enumerate(row):
                if falseOnlyField[r][c] in [
                    Minefield.pathSymbol,
                    Minefield.mineSymbol,
                    Minefield.dangerZoneSymbol,
                ]:
                    falseOnlyField[r][c] = Minefield.emptySymbol
        if print:
            print("  -  " * self.length)
            print("Displaying False Mines Only")
            for value in falseOnlyField:
                print()
                print(value)
            print("Displaying False Mines Only")
            print("  -  " * self.length)
        return falseOnlyField

    def displayOnlyPaths(self, print: bool = True):
        pathsOnlyField = deepcopy(self.minefield)
        for r, row in enumerate(pathsOnlyField):
            for c, col in enumerate(row):
                if pathsOnlyField[r][c] in [
                    Minefield.mineSymbol,
                    Minefield.dangerZoneSymbol,
                    Minefield.falseMineSymbol,
                ]:
                    pathsOnlyField[r][c] = Minefield.emptySymbol
        if print:
            print("  -  " * self.length)
            print("Displaying Paths Only")
            for value in pathsOnlyField:
                print()
                print(value)
            print("Displaying Paths only")
            print("  -  " * self.length)
        return pathsOnlyField

    def get(self):
        return self.minefield

    def __str__(self):
        for value in self.minefield:
            print()
            print(value)
        return ""

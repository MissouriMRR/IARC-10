import random

#Returns list of mine locations
def GenerateMineList(numM=6000, tRows=80, tCols=300):

    mineList = []

    # Generate mines
    for k in range(numM):
        row, col = random.randrange(tRows), random.randrange(tCols)

    # Avoid placing on existing mine
    while [row, col] in mineList:
        row, col = random.randrange(tRows), random.randrange(tCols)

    mineList.append([row, col])

    return mineList

#Returns list of mine locations as well as print a minefield 2d array
def GenerateMineField(numM=6000, tRows=80, tCols=300, dx=3, dy=3):
    
    # Parameters
    mine, danger, safe = 'M', 'm', '-'

    # Grid settings
    minefield = [[safe for _ in range(tCols)] for _ in range(tRows)]
    mineList = []

    # Generate mines
    for k in range(numM):
        row, col = random.randrange(tRows), random.randrange(tCols)

    # Avoid placing on existing mine
    while minefield[row][col] == mine:
        row, col = random.randrange(tRows), random.randrange(tCols)

    minefield[row][col] = mine
    mineList.append([row, col])

    # Mark danger zone for print
    for i in range(row - dx // 2, row + dx // 2 + 1):
        if not (0 <= i < tRows):
            continue
        for j in range(col - dy // 2, col + dy // 2 + 1):
            if not (0 <= j < tCols):
                continue
            if minefield[i][j] == safe:
                minefield[i][j] = danger

    # Print grid
    for row in minefield:
        print(" ".join(row))

    return mineList

def printMines(mineList):

    # Print mine list
    print("Mine locations:")
    print(mineList)

    return
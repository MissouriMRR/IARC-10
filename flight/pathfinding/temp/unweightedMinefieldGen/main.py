import random

# Parameters
numM = 30
mine, danger, safe = "M", "m", "-"
dx, dy = 3, 3

# Grid settings
tRows, tCols = 20, 20
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

# Print mine list
print("Mine locations:")
for m in mineList:
    print(m)

# Print grid
for row in minefield:
    print(" ".join(row))

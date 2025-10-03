import cv2
import numpy as np

origImg = cv2.imread('PFM1.png')

hsv = cv2.cvtColor(origImg, cv2.COLOR_BGR2HSV)

justGrass = hsv[50:600,50:600]
theMine = hsv[213:(213+25),700:(705+57)]

avgGrassH = 0
avgGrassS = 0
avgGrassV = 0

grassAmount = justGrass.shape
mineAmount = theMine.shape

avgMineH = 0
avgMineS = 0
avgMineV = 0

for row in justGrass:
    for item in row:
        avgGrassH += int(item[0])
        avgGrassS += int(item[1])
        avgGrassV += int(item[2])
    

for row in theMine:
    for item in row:
        avgMineH += int(item[0])
        avgMineS += int(item[1])
        avgMineV += int(item[2])

avgGrassH /= grassAmount[0] * grassAmount[1]
avgGrassS /= grassAmount[0] * grassAmount[1]
avgGrassV /= grassAmount[0] * grassAmount[1]

avgMineH /= mineAmount[0] * mineAmount[1]
avgMineS /= mineAmount[0] * mineAmount[1]
avgMineV /= mineAmount[0] * mineAmount[1]

print(f'Grass H: {avgGrassH}\nGrass S: {avgGrassS}\nGrass V: {avgGrassV}')
print(f'Mine H: {avgMineH}\nMine S: {avgMineS}\nMine V: {avgMineV}')
#print(grassAmount)

#cv2.imshow('orig',origImg)
#cv2.imshow('grass',justGrass)
#cv2.imshow('mine',theMine)

cv2.waitKey(0)
cv2.destroyAllWindows()

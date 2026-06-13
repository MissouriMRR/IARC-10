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

def return_avg(image):
    avgImage1 = 0
    avgImage2 = 0
    avgImage3 = 0

    amount = image.shape

    for row in image:
        for item in row:
            avgImage1 += int(item[0])
            avgImage2 += int(item[1])
            avgImage3 += int(item[2])

    avgImage1 /= amount[0] * amount[1]
    avgImage2 /= amount[0] * amount[1]
    avgImage3 /= amount[0] * amount[1]
    return [avgImage1,avgImage2,avgImage3]

#print(f'Grass H: {avgGrassH}\nGrass S: {avgGrassS}\nGrass V: {avgGrassV}')
#print(f'Mine H: {avgMineH}\nMine S: {avgMineS}\nMine V: {avgMineV}')

cv2.waitKey(0)
cv2.destroyAllWindows()

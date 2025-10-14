import random

#We may need to provide four arbitrary (fake) corners for calculation testing

#Make 2D array
tRows, tCols = 20, 20                    #Want 3937, 3937
minefield = [['o' for i in range(tCols)] for i in range(tRows)]
mineList = [[]]

#Make vars
numM = 30 #15,499,969 /441~35,147           #Want 40,000
row, col= 0, 0
mine, danger, safe = 'M', 'm', 'o'
dx, dy = 3, 3                                #Want 21, 21

#Generate mines
for k in range(numM):

    #Find available mine location
    while minefield[row][col] == mine:

        row = random.randrange(0,tRows)
        col = random.randrange(0,tCols)

    minefield[row][col] = mine
    mineList[k] = [row, col]

    #Build danger zone
    for i in range(row-dx//2, row+dx//2+1):
        if i<0 or i>tRows-1:
            continue
        else:
            for j in range(col-dy//2, col+dy//2+1):
                if j<0 or j>tCols-1 or (minefield[i][j] != safe):
                    continue
                else:
                    minefield[i][j]=danger

#print mineList
string = "["
for blowupDevice in mineList:
    row_str = str(blowupDevice) +

# #print minefield
width = len(str(tCols - 1))  # width based on max column number
header = " " * (width + 1) + " ".join(str(j).rjust(width) for j in range(tCols))
print(header)

for i in range(tRows):
    row_str = str(i).rjust(width) + " " + " ".join(cell.rjust(width) for cell in minefield[i])
    print(row_str)
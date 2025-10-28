import math as m
import numpy as np
import turtle as t

t.speed(0)
t.setworldcoordinates(0, 0, 3937, 3937)
t.penup()
screen = t.Screen()
screen.tracer(0)

# For Testing Purposes
np.random.seed(0)

numberOfMines = 1500

mineList = np.random.randint(3937, size=(numberOfMines+1, 2))
mineList[0] = (5000,5000)

# Format:
# mineConTrac[<reference mine postion in mineList>][0: 0-60, 1: 60-12, 2: 120-180, 3: 180-240, 4: 240-300, 5: 300-360]
mineConTrac = np.full((numberOfMines+1, 6), 0)

for i in range(1, len(mineList)):
    for y in range(i+1, len(mineList)-1):
        a = (mineList[y][1]-mineList[i][1])
        b = (mineList[y][0]-mineList[i][0])
        quad = 0

        if a < 0 and b < 0:
            quad = 180
        elif a < 0:
            quad = 360
        elif b < 0:
            quad = 180

        if ((0 < m.degrees(m.atan(a / b)) + quad <= 60) and (m.sqrt(a**2 + b**2) < m.sqrt((mineList[mineConTrac[i][0]][1]-mineList[i][1])**2 + (mineList[mineConTrac[i][0]][0]-mineList[i][0])**2))):
            mineConTrac[i][0] = y
            mineConTrac[y][3] = i
        elif ((60 < m.degrees(m.atan(a / b)) + quad <= 120) and (m.sqrt(a**2 + b**2) < m.sqrt((mineList[mineConTrac[i][1]][1]-mineList[i][1])**2 + (mineList[mineConTrac[i][1]][0]-mineList[i][0])**2))):
            mineConTrac[i][1] = y
            mineConTrac[y][4] = i
        elif ((120 < m.degrees(m.atan(a / b)) + quad <= 180) and (m.sqrt(a**2 + b**2) < m.sqrt((mineList[mineConTrac[i][2]][1]-mineList[i][1])**2 + (mineList[mineConTrac[i][2]][0]-mineList[i][0])**2))):
            mineConTrac[i][2] = y
            mineConTrac[y][5] = i
        elif ((180 < m.degrees(m.atan(a / b)) + quad <= 240) and (m.sqrt(a**2 + b**2) < m.sqrt((mineList[mineConTrac[i][3]][1]-mineList[i][1])**2 + (mineList[mineConTrac[i][3]][0]-mineList[i][0])**2))):
            mineConTrac[i][3] = y
            mineConTrac[y][0] = i
        elif ((240 < m.degrees(m.atan(a / b)) + quad <= 300) and (m.sqrt(a**2 + b**2) < m.sqrt((mineList[mineConTrac[i][4]][1]-mineList[i][1])**2 + (mineList[mineConTrac[i][4]][0]-mineList[i][0])**2))):
            mineConTrac[i][4] = y
            mineConTrac[y][1] = i
        elif ((300 < m.degrees(m.atan(a / b)) + quad <= 360) and (m.sqrt(a**2 + b**2) < m.sqrt((mineList[mineConTrac[i][5]][1]-mineList[i][1])**2 + (mineList[mineConTrac[i][5]][0]-mineList[i][0])**2))):
            mineConTrac[i][5] = y
            mineConTrac[y][2] = i

    t.goto(mineList[i])
    t.dot(15, "red")
t.pensize(1)
for i in range(1, len(mineConTrac)):
    for y in range(len(mineConTrac[i])):
        if (mineConTrac[i][y] == 0):
            t.goto(mineList[i])
            t.pendown()
            if y == 0:
                t.goto(3937, mineList[i][1] + (3937 - mineList[i][0]) * 0.5)
            elif y == 1:
                t.goto(mineList[i][0], 3937)
            elif y == 2:
                t.goto(0, mineList[i][1] + mineList[i][0] * 0.5)
            elif y == 3:
                t.goto(0, mineList[i][1] - mineList[i][0] * 0.5)
            elif y == 4:
                t.goto(mineList[i][0], 0)
            elif y == 5:
                t.goto(3937, mineList[i][1] - (3937 - mineList[i][0]) * 0.5)
            t.penup()
            continue
        t.goto(mineList[i])
        t.pendown()
        t.goto(mineList[mineConTrac[i][y]])
        t.penup()

screen.update()
t.done()
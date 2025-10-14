import math as m
import numpy as np
import turtle as t
import sys

def circle_mask(arraySize, circleCenter, circleRad):
    hight, width = arraySize
    y, x = np.ogrid[:hight, :width]
    mask = np.sqrt((x - circleCenter[0])**2 + (y - circleCenter[1])**2) <= circleRad
    return mask

# Configurations
t.colormode(255) # Sets the color mode to rgb values
np.random.seed(0) # Seed setter
numberOfMines = 5 # Total number of mine to generate
mineHeat = 50 # The heat value the a mine will have (max heat value)
fieldSize = 300 # The size of the square field
t.speed(0) # Maxes the turtle speed
t.setworldcoordinates(0, 0, fieldSize, fieldSize) # Creates a configured turtle window
t.pensize(5)
t.penup()
t.hideturtle()
screen = t.Screen()
screen.tracer(0) # Turns off the turtle tracer

# Mine generation
mineList = np.random.randint(fieldSize, size=(numberOfMines, 2))\

# Discovery Layer Creation
discLayer = np.full((fieldSize, fieldSize), 1)

# Layers the heat onto the the discovery layer
for y in range(mineHeat-1):
    for i in range(len(mineList)):
        discLayer[circle_mask(discLayer.shape, mineList[i], mineHeat-1-y)] = y+2
        t.goto(mineList[i])
        t.color((255, int(255-(255/mineHeat)*(y+1)), int(255-(255/mineHeat)*(y+1))))
        t.goto(t.xcor(), t.ycor()-(mineHeat-1-y))
        t.pendown()
        t.circle((mineHeat-1-y))
        t.penup()
        t.goto(mineList[i])
        t.dot(5, "black")

# Vestigial Testing Code Used to See whats in the array exactly
#
#    for i in range(len(discLayer)):
#        for y in range(len(discLayer[i])):
#            t.goto(y, i)
#            t.color((255, int(255-(255/mineHeat)*(discLayer[i][y]-1)), int(255-(255/mineHeat)*(discLayer[i][y]-1))))
#            t.dot(2)

print("done")
t.update()
t.done()
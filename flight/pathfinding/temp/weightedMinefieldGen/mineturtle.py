import turtle as t
from flight.pathfinding.weightedMinefieldGen.minefield import *

length = 70
width = 70
t.setworldcoordinates(0, 0, length, width)
t.penup()
t.pensize(1)
screen = t.Screen()
screen.tracer(0)
screen.setup(700, 650)
t.speed(0)
field = Minefield(length, width, False)
field.createPath([length - 1, (width - 1) // 3])
field.createPath([length - 1, (width - 1)])
field.generateMines(1000)
# field.generateOtherObstacles(5) Do not run this yet, it will not work

for y, row in enumerate(field.get()):
    for x, col in enumerate(field.get()[0]):
        if field.get()[y][x] in [Minefield.pathSymbol]:
            t.color("green")
        elif field.get()[y][x] in [Minefield.mineSymbol]:
            t.color("black")
        elif field.get()[y][x] in [Minefield.dangerZoneSymbol]:
            t.color("red")
        elif field.get()[y][x] in [Minefield.otherObstaclesSymbol]:
            t.color("brown")
        t.goto(x, y)
        t.begin_fill()
        t.goto(x, y + 1)
        t.goto(x + 1, y + 1)
        t.goto(x + 1, y)
        t.goto(x, y)
        t.end_fill()
# field.displayOnlyMines()
# field.displayOnlyPaths()
t.done()

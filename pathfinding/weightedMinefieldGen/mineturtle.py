import turtle as t
from minefield import *
"""
This will display the field using Turtle.

**Weighted Minefield Gen as in the paths have weights
  that can be changed that influence which direction 
  is chosen when creating paths.**
"""

length = 100
width = 100
t.setworldcoordinates(0, 0, length, width)
t.penup()
t.pensize(1)
screen = t.Screen()
screen.tracer(0)
screen.setup(700,650) # Adjust accordingly to fit the window to the field. 
t.speed(0)
field = Minefield(length,width,True)

# Paths created may intersect a lot
# False paths(Random green spots along the edge or sudden long stretches) may exist.
# Not intentional but I guess a useful bug ;)
# Illegal Mines may be generated
# Not intentional, it is a bug :( 

field.createPath([length-1,(width-1)//3])
field.createPath([length-1,(width-1)])
field.createPath([length-1,(width-1)//2])
field.createPath([length-1,0])

field.generateMines(1000) # May not generate the exact amount of mines.

# Use turtle to display the list in color
"""
# Green - Safe Path
# Black - Mine
# Red - Dangerzone/Mine radius
"""
for y,row in enumerate(field.get()):
    for x,col in enumerate(field.get()[0]):
        if field.get()[y][x] in [Minefield.pathSymbol]:
            t.color("green")
        elif field.get()[y][x] in [Minefield.mineSymbol]:
            t.color("black")
        elif field.get()[y][x] in [Minefield.dangerZoneSymbol]:
            t.color("red")
        t.goto(x,y)
        t.begin_fill()
        t.goto(x,y+1)
        t.goto(x+1,y+1)
        t.goto(x+1,y)
        t.goto(x,y)
        t.end_fill()
t.done()


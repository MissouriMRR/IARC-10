import turtle as t
from minefield import *
"""
This will display the field using Turtle.

**Weighted Minefield Gen as in the paths have weights
  that can be changed that influence which direction 
  is chosen when creating paths.**
"""

displayPath = True
displayMines = True
displayFalseMines = True

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
# Orange - False Mine
"""
cell_size = 0.8
mineCount = 0
dangerCount = 0
falseMineCount = 0
safePathCellCount = 0

for y,row in enumerate(field.get()):
    for x,col in enumerate(field.get()[0]):
        if field.get()[y][x] in [Minefield.pathSymbol] and displayPath:
            t.color("green")
            safePathCellCount += 1
        elif field.get()[y][x] in [Minefield.mineSymbol] and displayMines:
            t.color("black")
            mineCount += 1
        elif field.get()[y][x] in [Minefield.dangerZoneSymbol] and displayMines:
            t.color("red")
            dangerCount += 1
        elif field.get()[y][x] in [Minefield.falseMineSymbol] and displayFalseMines:
            t.color("dark orange")
            falseMineCount += 1
        else:
            dangerCount += 1
                
        t.goto(x,y)
        t.begin_fill()
        t.goto(x,y+cell_size)
        t.goto(x+cell_size,y+cell_size)
        t.goto(x+cell_size,y)
        t.goto(x,y)
        t.end_fill()
t.done()
print("M I N E F I E L D   S T A T I S T I C S :")
print(f'Number of Mines Successfully Created:\n{mineCount}')
print(f'Number of Mines False Mines Successfully Created:\n{falseMineCount}')
print(f'Percentage of Field That is Safe:\n{(safePathCellCount/(length*width))*100:.2f}%')
print(f'Percentage of Field That is Dangerous(Mines and Danger Zones):\n{((mineCount+dangerCount)/(length*width))*100:.2f}%')
print(f'Percentage of Field That are False Mines:\n{(falseMineCount/(length*width))*100:.2f}%')
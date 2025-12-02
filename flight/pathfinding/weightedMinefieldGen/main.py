from flight.pathfinding.weightedMinefieldGen.minefield import *

length, width =30,30 # Change to 3737x3737 later

field = Minefield(length,width,True)
"""
Please compatible numbers is reconmended,
unless you want to fill everything with mines.
For example, you cannot place 100 mines on a grid of a size less than a 10x10,
and thats not including grid spaces occupied by safe paths.
If you do, there will be no empty spaces

Same goes for start points of safe paths.
"""

# field.createPath([length-1,width//4]) # Starts in bottom left quarter
# field.createPath([length-1,0]) # Starts in bottom left corner
# field.createPath([length-1,width - (width//3)]) # Starts in bottom right third
field.createPath([length-1,width-1]) # Starts in bottom right corner

field.generateMines(500,2)
field.generateOtherObstacles(2)
field.displayOnlyMines()
print()
field.displayOnlyPaths()
print()
print(field)

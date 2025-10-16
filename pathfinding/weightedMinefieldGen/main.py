from minefield import *
"""
This will display the field grid in the form of a 2D List.
"""
length, width =30,30 # To be 3737x3737, it is 30x30 for it to actually fit in the console 

field = Minefield(length,width,True)



# field.createPath([length-1,width//4]) # Starts in bottom left quarter
# field.createPath([length-1,0]) # Starts in bottom left corner
# field.createPath([length-1,width - (width//3)]) # Starts in bottom right third
field.createPath([length-1,width-1]) # Starts in bottom right corner

field.generateMines(500,2) 

field.displayOnlyFalseMines()
print()
field.displayOnlyMines()
print()
field.displayOnlyPaths()
print()
print(field)

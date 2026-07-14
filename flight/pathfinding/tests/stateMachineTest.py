from flight.pathfinding.node_generation.node_generation import Field, Mine, Node
from flight.pathfinding.path_calculation import Graph
from flight.pathfinding.utils.coord_convert import SimToLatLonTransformer as coordCon
from random import randint, seed, uniform
import math
import time
#WHAT THE COMPETITION STATE MACHINE WILL DO

startLocation = (100,-105)
fieldCorners=((100, -105), (100, 105), (-100, 105), (-100, -105)) #get from coord converter

mineRadius = 3 #Calculated based on size of image that can consistently detect mines. 
#Information about width of image in feet that the camera can detect mines at is given.
#We put into arb coord conversion to get relevent mineRadius in arbitrary units. 



field = 

while True:

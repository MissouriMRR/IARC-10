"""
The protoMine class is the precursor to a mine being added to both the block field and node field

Use the protomine test case to get a better idea of what the protomine does

Because polygon obstacle vertices depend on the block structure of the node, we calculate
both in one class. Then a function in block field/node field accepts and applies the protomine
to their respective fields at their global location

Yes it is intended behavior to generate vertices outside the given bounds, nodegraph handles out of bounds checking
"""

from flight.pathfinding.blockField.field_grid import *
SQUARE_SIDE_LENGTH_FT=2
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Circle
from matplotlib.lines import Line2D
from flight.pathfinding.common import nodeDirection
from flight.pathfinding.nodeField.BlockMineNode import BlockMineNode

class protoMine():
    #Saftey radius by which nearby grids may also contain this mine.

    def circle_rect_intersects(
        cx: float,
        cy: float,
        r: float,
        rx: float,
        ry: float,
        l: float,
    ):
        """
        Determine whether a circle intersects a square.

        Parameters:
            cx: x-coordinate of the circle center
            cy: y-coordinate of the circle center
            r: radius of the circle
            rx: x-coordinate of the square's top-left corner
            ry: y-coordinate of the squares's top-left corner
            l: side length of the square
        """


        # Closest x on square
        closestX = rx
        if(abs(cx-(rx+l))<=abs(cx-rx)):
            closestX = rx + l

        elif cx < rx:
            closestX = rx

        # Closest y on square
        closestY = ry
        if(abs(cy-(ry+l))<=abs(cy-ry)):
     
            closestY = ry + l
        elif cy < ry:
            closestY = ry

        # Distance from circle center to closest point
        distX = cx - closestX
        distY = cy - closestY
        distance = (distX**2 + distY**2) ** 0.5

        #non corner, but in circle check edge case:
        touchingY=abs(cy-closestY)<=r and abs(rx+l/2-cx)<=l/2
        touchingX=abs(cx-closestX)<=r and abs(ry+l/2-cy)<=l/2
        return distance <= r or touchingX or touchingY

    def __init__(self,safteyRadius,mineLatLon,centerGridOffset):
        self.mineLatLon=mineLatLon
        self.centerGridOffset=centerGridOffset
        self.safteyBlockRadius=round(safteyRadius)
        self.placeholderGridSideLength=safteyRadius*2+1
        self.blockMatrix = np.zeros((self.placeholderGridSideLength, self.placeholderGridSideLength), dtype=int).tolist()
        self.blockMatrix[self.placeholderGridSideLength//2][self.placeholderGridSideLength//2]=1
        self.mineLocation=tuple(self.centerOfBlock(self.placeholderGridSideLength//2,self.placeholderGridSideLength//2))

        self.nodeVertices=[]
        self.generateBlocks()
        self.generateNodes()

    def centerOfBlock(self,x,y,offsetX=0,offsetY=0):
        yCoord=(y)*SQUARE_SIDE_LENGTH_FT+SQUARE_SIDE_LENGTH_FT/2 + self.centerGridOffset[1]
        xCoord=x*SQUARE_SIDE_LENGTH_FT+SQUARE_SIDE_LENGTH_FT/2 + self.centerGridOffset[0]
        return [xCoord+offsetX,yCoord+offsetY]
    def generateBlocks(self):

        for y in range(len(self.blockMatrix)):
            for x in range(len(self.blockMatrix[y])):
                xCoord=x*SQUARE_SIDE_LENGTH_FT + self.centerGridOffset[0]
                yCoord=(y)*SQUARE_SIDE_LENGTH_FT + self.centerGridOffset[1]
                self.blockMatrix[y][x] = int(protoMine.circle_rect_intersects(self.mineLocation[0], self.mineLocation[1], self.safteyBlockRadius, xCoord, yCoord, SQUARE_SIDE_LENGTH_FT))


    #Generate blocks must be called prior
    def generateNodes(self):
        rows = len(self.blockMatrix)
        cols = len(self.blockMatrix[0]) if rows else 0
        wrappingVertices=[]
        wrappingDirections=[]
        for y in range(rows):
            for x in range(cols):
                if(self.blockMatrix[y][x]==1):
                    #Left
                    if(x==0 or self.blockMatrix[y][x-1]==0):
                        
                        wrappingVertices.append(self.centerOfBlock(x-1,y,-0.05,0))
                        wrappingDirections.append(nodeDirection.LEFT)
                    #Down
                    if(y==0 or self.blockMatrix[y-1][x]==0):
                        wrappingVertices.append(self.centerOfBlock(x,y-1,0,-0.05))
                        wrappingDirections.append(nodeDirection.DOWN)
                    #Right
                    if(x==cols-1 or self.blockMatrix[y][x+1]==0):
                        wrappingVertices.append(self.centerOfBlock(x+1,y,0.05,0))
                        wrappingDirections.append(nodeDirection.RIGHT)
                    #UP
                    if(y==rows-1 or self.blockMatrix[y+1][x]==0):
                        wrappingVertices.append(self.centerOfBlock(x,y+1,0,0.05)) 
                        wrappingDirections.append(nodeDirection.UP)
        if(wrappingVertices==[]):
            return []
        vertexIndicies=ConvexHull(wrappingVertices).vertices
        self.nodeVertices=[]
        for i in vertexIndicies:
            self.nodeVertices.append(BlockMineNode(wrappingVertices[i][0],wrappingVertices[i][1],wrappingDirections[i]))
        return self.nodeVertices
    def visualize(self):
        fig, ax = plt.subplots()
        #self.blockMatrix[6][3]=1
        plt.imshow(self.blockMatrix, cmap='gray',origin='lower', extent=[0, self.placeholderGridSideLength*SQUARE_SIDE_LENGTH_FT, 0, self.placeholderGridSideLength*SQUARE_SIDE_LENGTH_FT])
        ax.add_patch(Circle((self.mineLocation[0], self.mineLocation[1]), self.safteyBlockRadius, color='green', alpha=0.5))
        ax.set_facecolor("grey")
        poly = Line2D([vertex.x for vertex in self.nodeVertices], [vertex.y for vertex in self.nodeVertices], color='orangered', linewidth=3)
        for i in self.nodeVertices:
            ax.plot(i.x, i.y, marker='o', markersize=12, linestyle='-', color='blue')
        ax.add_line(poly)


        for y in range(len(self.blockMatrix)):
            poly=Line2D([0,self.placeholderGridSideLength*SQUARE_SIDE_LENGTH_FT],[y*SQUARE_SIDE_LENGTH_FT,y*SQUARE_SIDE_LENGTH_FT],color='red',linewidth=1)
            ax.add_line(poly)
        for x in range(len(self.blockMatrix[0])):
            poly=Line2D([x*SQUARE_SIDE_LENGTH_FT,x*SQUARE_SIDE_LENGTH_FT],[0,self.placeholderGridSideLength*SQUARE_SIDE_LENGTH_FT],color='red',linewidth=1)
            ax.add_line(poly)
        
                

            
        plt.show()

        

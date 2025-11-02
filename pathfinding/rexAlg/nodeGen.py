import matplotlib.pyplot as pyplot
import numpy as np
import random
from itertools import combinations
"""
When using the attributes/methods, refer to the object unless intentionally accessing a class variable.

KEY MINE ATTRIBUTES/METHODS:
 - Mine.getPos() -> (x,y) position on field of the Mine
 - Mine.getNodes() -> [Node,Node,...] A list of all mine's child Nodes

KEY NODE ATTRIBUTES/METHODS:

 - Node.terminated -> True/False (Very IMPORTANT, this node should not be counted if terminated is True)

 - Node.type() -> "internal/external primary/secondary" (Either internal OR external AND primary OR secondary)
 - Node.parentMine -> Mine object that the node is on
 - Node.getPos() -> (x,y) position on field of the Node
 - Node.distanceFromNode(node) -> distance (Simple distance between nodes)

KEY FIELD ATTRIBUTES/METHODS:
 - Field.addMine(centerX,centerY,radius) -> Adds a mine to the field and creates and connects Nodes accordingly
 - Field.plotField() -> Using matplotlib, plots the field.

---------------------------------------------------------------------
Use the Field class to generate mines and their nodes.
Ex:

    field = Field(-100,150,-100,150)

    Field(xMin,xMax,yMin,yMax)
    Field Setup:
     - xMin,xMax : The fields x axis range
     - yMin, yMax : The fields y axis range
"""

# Field generates nodes off of mines, temporarily generates mines too
class Field:
    def __init__(self,xMin,xMax,yMin,yMax):
        self.xMin = xMin
        self.yMin = yMin
        self.xMax = xMax
        self.yMax = yMax
        self.mines = []
    
    def addMine(self,centerX,centerY,radius):
        self.mines.append(Mine(centerX,centerY,radius))
        mineCombo = list(combinations(self.mines,2)) # group unique pairs of mines together
        for pair in mineCombo:
            mine = pair[0]
            target = pair[1]
            
            if target not in mine.connectedMines and mine not in target.connectedMines:
                # Mine internal nodes
                mineInternPrimary = Node(mine,target,True,True)
                mineInternSecond = Node(mine,target,True,False)      
                mine.addNode(mineInternPrimary)
                mine.addNode(mineInternSecond)

                # Mine External nodes
                mineExternPrimary = Node(mine,target,False,True)
                mineExternSecond = Node(mine,target,False,False)
                mine.addNode(mineExternPrimary)
                mine.addNode(mineExternSecond)

                # target internal nodes
                targetInternPrimary = Node(target,mine,True,False)
                targetInternSecond = Node(target,mine,True,True)
                target.addNode(targetInternPrimary)
                target.addNode(targetInternSecond)
                # target external nodes
                targetExternPrimary = Node(target,mine,False,False)
                targetExternSecond = Node(target,mine,False,True)
                target.addNode(targetExternPrimary)
                target.addNode(targetExternSecond)

                # Connect Nodes
                mineInternPrimary.connectNode(targetInternSecond)
                mineInternSecond.connectNode(targetInternPrimary)

                mineExternPrimary.connectNode(targetExternPrimary)
                mineExternSecond.connectNode(targetExternSecond)
        
        # Terminate pair of Nodes if their connection is blocked by another mine
        for node in Node.nodes:
            connectedNode = node.connectedNode
            x1 = node.x
            y1 = node.y
            x2 = connectedNode.x
            y2 = connectedNode.y
            for mine in self.mines:
                x3 = mine.x
                y3 = mine.y

                # Fraction of segment between nodes that the mine lands perpendicular to segment
                uNumerator = ((x3 - x1)*(x2 - x1)) + ((y3 - y1)*(y2 - y1))
                uDenominator = ((x1-x2)**2) + ((y1-y2)**2)                
                u = np.clip(uNumerator/uDenominator,0,1) # Restrict to the constraints of a segment

                # Adjust accordingly, determines how close a mine can be to a node before the node terminates
                uMin = 0.03
                uMax = 0.95
                if uMin <= u <= uMax:
                    # Point on segment that is tangent and perpendicular to mine
                    tangePoint = (x1 + (u*(x2-x1)),y1 + (u*(y2-y1)))
                    
                    distanceFromMine = np.sqrt((mine.x - tangePoint[0])**2+(mine.y - tangePoint[1])**2)
                    if distanceFromMine < radius:
                        node.terminated = True
                        
    # Plotting using matplotlib
    def plotField(self):
        plt = pyplot
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        plt.xlim(self.xMin,self.xMax)
        plt.ylim(self.yMin,self.yMax)
        
        # Create a list of circles representing mines, centered to their correlated mine's center
        circles = [plt.Circle(self.mines[i].getPos(),self.mines[i].getRadius(),color=(np.random.random(),np.random.random(),np.random.random())) for i in range(len(self.mines))]
        
        # Plot the mines
        for circle in circles:
            ax.add_patch(circle)

        # Plot the nodes
        nodeSymbol = ''
        for node in Node.nodes:
            if not node.plotted and not node.terminated:
                try:
                    plt.plot([node.x,node.connectedNode.x],[node.y,node.connectedNode.y],nodeSymbol)
                except AttributeError:
                    plt.plot([node.x],[node.y],nodeSymbol)
        plt.show()

"""MATH STUFF"""
# Mine class keeps track of mine position and radii      
class Mine:
    numMines = 0
    mines = []
    def __init__(self,centerX,centerY,radius,name:str=''):
        Mine.numMines += 1
        if len(name) > 0:
            self.name = name
        else:
            self.name = f'Mine ID: ' + str(self.numMines)
        self.x, self.y, self.radius = np.round(centerX,2) , np.round(centerY,2), np.round(radius,2)
        # Node storage
        self.__nodes = []
        self.__primary = []
        self.__secondary = []
        self.__internal = []
        self.__external = []
        self.connectedMines = []
        Mine.mines.append(self)
        
        
    def getPos(self):
        return self.x,self.y
    def getRadius(self):
        return self.radius
    def getNodes(self,type:str=''):
        if type == None or type == '':
            return self.__nodes
        elif type.lower() == 'primary':
            return self.__primary
        elif type.lower() == 'secondary':
            return self.__secondary
        elif type.lower() == 'internal':
            return self.__internal
        elif type.lower() == "external":
            return self.__external
        else:
            return 'invalid node type'
    def addNode(self,node:"Node"):
        self.__nodes.append(node)
        internal = node.internal
        primary = node.primary
        if internal:
            self.__internal.append(node)
        else:
            self.__external.append(node)
        
        if primary:
            self.__primary.append(node)
        else:
            self.__secondary.append(node)

    def __str__(self):
        return self.name
    def __repr__(self):
        return self.__str__()

# Node class keeps track of node positions
class Node:
    nodeNum = 0
    nodes = []
    def __init__(self,parentMine:"Mine",targetMine:"Mine",internal:bool=True,primary:bool=True,name:str=''):
        Node.nodeNum += 1
        if len(name) < 1:
            self.name = "Node ID: "+ str(Node.nodeNum)
        else:
            self.name = name 
        self.parentMine = parentMine
        self.connectedNode = None
        self.x = 0.0
        self.y = 0.0
        self.connected = False
        self.plotted = False # To prevent hopefully duplicate plotting
        self.__targetMine = targetMine
        self.terminated = False

        # categorize nodes
        if internal and primary:
            self.type = 'internal primary'
        elif internal and not primary:
            self.type = 'internal secondary'
        elif not internal and primary:
            self.type = 'external primary'
        elif not internal and not primary:
            self.type = 'external secondary'

        d = np.sqrt((parentMine.x-targetMine.x)**2+(parentMine.y-targetMine.y)**2) # Algabraic Distance Formula

        self.internal = internal
        self.primary= primary # Primary node is the first node where it is placed 
                              # (typically towards the top of the circle)
        # Create Angle Offset(relative to target mine). It changes slightly depending on mines' positions
        # Formula is: arccos(x1-x2)/d+pi
        if parentMine.y > targetMine.y:
               offsetAngle =  np.arccos(np.clip((parentMine.x-targetMine.x)/d,-1,1))+np.pi
        elif parentMine.y < targetMine.y:
            offsetAngle = -np.arccos(np.clip((parentMine.x-targetMine.x)/d,-1,1))+np.pi
        elif parentMine.y == targetMine.y:
            if parentMine.x < targetMine.x:
                offsetAngle =  np.arccos(np.clip((parentMine.x-targetMine.x)/d,-1,1))+np.pi
            elif parentMine.x > targetMine.x:
                offsetAngle = -np.arccos(np.clip((parentMine.x-targetMine.x)/d,-1,1))+np.pi

        # Offset Angle is the same for internal and external bitangents
        if internal:
            # Create internal angle
            internalAngle = np.arccos(np.clip(((parentMine.radius)+(targetMine.radius))/d,-1,1))
            
            if primary:
                self.x = ((parentMine.radius) * np.cos(internalAngle+offsetAngle)) + parentMine.x
                self.y = ((parentMine.radius) * np.sin(internalAngle+offsetAngle)) + parentMine.y
            else:
                self.x = (parentMine.radius) * np.cos(internalAngle-(offsetAngle)) + parentMine.x
                self.y = (parentMine.radius) * np.sin(internalAngle-(offsetAngle)+np.pi) + parentMine.y
        else:
            # Create external angle
            externalAngle = np.arccos(np.abs(parentMine.radius-targetMine.radius)/d)

            if primary:
                self.x = ((parentMine.radius) * np.cos(externalAngle+offsetAngle)) + parentMine.x
                self.y = ((parentMine.radius) * np.sin(externalAngle+offsetAngle)) + parentMine.y
            else:
                self.x = (parentMine.radius) * np.cos(externalAngle-(offsetAngle)) + parentMine.x
                self.y = (parentMine.radius) * np.sin(externalAngle-(offsetAngle)+np.pi) + parentMine.y
        
        self.x = round(self.x,3)
        self.y = round(self.y,3)
        Node.nodes.append(self)
        
    def distanceFromNode(self,node:"Node"):
        distance = 0
        if len(self.connectedNodes) > 0 and node in self.connectedNodes:
            distance = np.sqrt((self.x-node.x)**2+(self.y-node.y)**2)
        return distance

    def status(self):
        result = []
        if self.terminated:
            result.append("Node terminated")
        else:
            result.append("Node exists")
        
        if self.connected:
            result.append("Node connected")
        else:
            result.append("Node not connected")
        return result

    def connectNode(self,node:"Node"):
        self.connectedNode = node
        node.connectedNode = self
        self.connected = True
        node.connected = True
        if node.parentMine not in self.parentMine.connectedMines and self.parentMine not in node.parentMine.connectedMines:
            self.parentMine.connectedMines.append(node.parentMine)
            node.parentMine.connectedMines.append(self.parentMine)
        return node

    def getPos(self):
        return (round(float(self.x),3),round(float(self.y),3))

    def getTargetMine(self):
        return self.__targetMine
    def __str__(self):
        return self.name
    def __repr__(self):
        return self.__str__()

numMines = 15
xMin = -150
xMax = 150
yMin = -150
yMax = 150
radius = 16
position = [0,0]
field = Field(xMin,xMax,yMin,yMax)
debug = False

# Mine generation
if not debug:
    for num in range(numMines):
        while True: # To make sure generated mines arent clipping off the edges of the field
            position[0], position[1] = random.randint(xMin,xMax+1),random.randint(yMin,yMax+1)
            invalidPosition = False
            for mine in field.mines:
                if (mine.getPos()[0] - 2*radius <= position[0] <= mine.getPos()[0] + 2*radius) and (mine.getPos()[1] - 2*radius <= position[1] <= mine.getPos()[1] + 2*radius):
                    invalidPosition = True
                    break
            if invalidPosition:
                continue
            if position[0] <= xMin + radius or position[0] >= xMax - radius or position[1] <= yMin + radius or position[1] >= yMax - radius:
                continue
            break
        field.addMine(position[0],position[1],radius)

if debug:
    for mine in field.mines:
        print(mine,'connected to',','.join(m.__str__() for m in mine.connectedMines))
    print(Node.nodes)

field.plotField()


import matplotlib.pyplot as pyplot
import numpy as np
import random
from itertools import combinations
import time
from sys import getrefcount
import gc


"""
When using the attributes/methods, refer to the object unless intentionally accessing a class variable.

KEY MINE ATTRIBUTES/METHODS:
 - Mine.getPos() -> (x,y) position on field of the Mine
 - Mine.getNodes() -> [Node,Node,...] A list of all mine's child Nodes, can be accessed directly with Mine.nodes
 - Mine.connectedMines -> [Mine,Mine,...] A list of other mines connected to the Mine through at least one Node

KEY NODE ATTRIBUTES/METHODS:
 - Node(primaryType) -> Constructor key parameter, either 'default','start', or 'end', must 
                        explicitely define primaryType if not a default node when constructing

 - Node.terminated -> True/False (Very IMPORTANT, this node should not be counted if terminated is True)
                    |-For some reason, trying to completely delete all references to a Node breaks stuff-| 

 - Node.type() -> "internal/external primary/secondary" (Either internal OR external AND primary OR secondary)
 - Node.parentMine -> Mine object that the node is primarily on
 - Node.connectedNodes -> [Node,Node,...] A list of connected nodes, excluding the Node itself.
                                          Usually a list length of one as a result of connecting bitangents,
                                         unless furthur connections are established.
 - Node.connectNode(Node) -> Connects nodes by appending to each other's connectedNodes list.
 - Node.status() -> ["terminated"/"exists", "connected"/"disconnected"]
                     A list of strings describing the status of a single node
 - Node.getPathType(Node) -> ["line"/"arc", "established"/"unestablished","bitangent"/"normal"] 
                              A list of strings describing the kind of connection each node has to another
 - Node.getPos() -> (x,y) position on field of the Node
 - Node.distanceFromNode(Node) -> distance between nodes, either a straight line distance or
                                  arc length if nodes share the same parent mine

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
    
    def addMine(self,centerX,centerY,radius,color:str=''):
        newMine = Mine(centerX,centerY,radius,color=color)
        mineCombo = [[newMine, mine] for mine in Mine.mines[:-1]]
        for pair in mineCombo:
            mine = pair[0]
            target = pair[1]
            
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

        """Terminate pair of Nodes in certain conditions"""
        # Check newMine Nodes intersecting other mines
        for node in newMine.nodes:

            # Generally, the first connected Node is an established bitangent connectoin 
            connectedNode = node.connectedNodes[0] 

            if not node.terminated and not connectedNode.terminated:
                x1 = float(node.x)
                y1 = float(node.y)
                x2 = float(connectedNode.x)
                y2 = float(connectedNode.y)

                for mine in Mine.mines:
                    x3 = mine.x
                    y3 = mine.y
                    
                    # Fraction of segment between nodes that the mine lands perpendicular to segment
                    uNumerator = ((x3 - x1)*(x2 - x1)) + ((y3 - y1)*(y2 - y1))
                    uDenominator = ((x1-x2)**2) + ((y1-y2)**2)
                    if uDenominator == 0:
                        u = 0
                    else:
                        u = np.clip(uNumerator/uDenominator,0,1) # Restrict to the constraints of a segment

                    # Adjust accordingly, determines how close a mine can be to a node before the node terminates
                    uMin = 0.03
                    uMax = 0.98
                    if uMin <= u <= uMax:
                        # Point on segment that is tangent and perpendicular to mine
                        tangePoint = (x1 + (u*(x2-x1)),y1 + (u*(y2-y1)))
                        
                        distanceFromMine = np.sqrt((mine.x - tangePoint[0])**2+(mine.y - tangePoint[1])**2)
                        if distanceFromMine < radius:
                            node.terminated = True
                            connectedNode.terminated = True

                # Checks for nodes within another mine's radius
                for mine in Mine.mines:
                    if mine != node.parentMine:
                        if (mine.x - radius <= node.x <= mine.x + radius and
                            mine.y - radius <= node.y <= mine.y + radius):
                            node.terminated = True
                            node.connectedNodes[0].terminated = True
            
                    
                    
        # Check all other nodes Excluding newly created nodes if they intersect newly created Mine
        for node in [n for n in Node.nodes if n not in newMine.nodes]:
            connectedNode = node.connectedNodes[0]
            if not node.terminated and not connectedNode.terminated:
                x1 = float(node.x)
                y1 = float(node.y)
                x2 = float(connectedNode.x)
                y2 = float(connectedNode.y)
                x3 = newMine.x
                y3 = newMine.y
                
                # Fraction of segment between nodes that the mine lands perpendicular to segment
                uNumerator = ((x3 - x1)*(x2 - x1)) + ((y3 - y1)*(y2 - y1))
                uDenominator = ((x1-x2)**2) + ((y1-y2)**2)
                if uDenominator == 0:
                    u = 0
                else:
                    u = np.clip(uNumerator/uDenominator,0,1) # Restrict to the constraints of a segment

                # Adjust accordingly, determines how close a mine can be to a node before the node terminates
                uMin = 0.03
                uMax = 0.98

                if uMin <= u <= uMax:
                    # Point on segment that is tangent and perpendicular to mine
                    tangePoint = (x1 + (u*(x2-x1)),y1 + (u*(y2-y1)))
                    
                    distanceFromMine = np.sqrt((mine.x - tangePoint[0])**2+(mine.y - tangePoint[1])**2)
                    if distanceFromMine < radius:
                        node.terminated = True
                        connectedNode.terminated = True
                
                # Check for nodes within another mine's radius
                for mine in Mine.mines:
                    if mine != node.parentMine:
                        if (mine.x - radius <= node.x <= mine.x + radius and
                            mine.y - radius <= node.y <= mine.y + radius):
                            node.terminated = True
                            node.connectedNodes[0].terminated = True
        
    def plotField(self):
        plt = pyplot
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        plt.xlim(self.xMin,self.xMax)
        plt.ylim(self.yMin,self.yMax)
        
        # Create a list of circles representing mines, centered to their correlated mine's center

        circles = [plt.Circle(Mine.mines[i].getPos(),Mine.mines[i].getRadius(),color=Mine.mines[i].color) for i in range(len(Mine.mines))]
        
        # Plot the mines
        for circle in circles:
            ax.add_patch(circle)

        # Plot the nodes
        nodeSymbol = '' # Empty string makes either lines or invisible points; otherwise points are displayed using the symbol
        for node in Node.nodes:
            if not node.plotted and not node.terminated:
                for connectedNode in node.connectedNodes:
                    try:
                        plt.plot([node.x,connectedNode.x],[node.y,connectedNode.y],nodeSymbol)
                    except AttributeError:
                        plt.plot([node.x],[node.y],nodeSymbol)
        plt.show()

"""MATH STUFF"""
# Mine class keeps track of mine position and radii      
class Mine:
    numMines = 0
    mines = []
    def __init__(self,centerX,centerY,radius,color:str='',name:str=''):
        Mine.numMines += 1
        if len(name) > 0:
            self.name = name
        else:
            self.name = f'Mine ID: ' + str(self.numMines)
        if len(color) > 0:
            self.color = color
        else:
            self.color = np.random.random(),np.random.random(),np.random.random()
        self.x, self.y, self.radius = np.round(centerX,2) , np.round(centerY,2), np.round(radius,2)
        # Node and connect Mines storage
        self.nodes = []
        self.connectedMines = []
        Mine.mines.append(self)
        
        
    def getPos(self):
        return self.x,self.y
    def getRadius(self):
        return self.radius
    def getNodes(self):
        return self.nodes
    def addNode(self,node:"Node"):
        Node.nodes.append(node)
        self.nodes.append(node)
        return node
    def getNumReferences(self):
        return getrefcount(self)

    def __str__(self):
        return self.name
    def __repr__(self):
        return self.__str__()

# Node class keeps track of node positions
class Node:
    nodeNum = 0
    nodes = []
    
    def __init__(self,parentMine:"Mine",targetMine:"Mine",internal:bool=True,primary:bool=True,name:str='',primaryType='default'):
        
        Node.nodeNum += 1
        if len(name) < 1:
            self.name = "Node ID: "+ str(Node.nodeNum)
        else:
            self.name = name
        
        if primaryType == 'default':
            self.parentMine = parentMine
            self.connectedNodes = []
            self.x = 0.0
            self.y = 0.0
            self.connected = False
            self.plotted = False # To prevent hopefully duplicate plotting
            self.__targetMine = targetMine
            self.terminated = False
            self.lowest = False # Is there any other nodes below self
                              # /\ Determines start/end connections \/
            self.highest = False # Is there any other nodes above self

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
                internalCos = (parentMine.radius + targetMine.radius)/d
                if internalCos < 1:
                    internalAngle = np.arccos(np.clip(((parentMine.radius)+(targetMine.radius))/d,-1,1))
                else:
                    self.terminated = True
                    return None
                
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
    def setPosition(self,x:float,y:float):
        self.x = x
        self.y = y
    # DISTANCE
    def distanceFromNode(self,node:"Node") -> float:
        distance = 0.0
        if len(self.connectedNodes) > 0 and node in self.connectedNodes:
            if node.parentMine == self.parentMine: # Nodes are on the same mine
                currentNodeDistance = np.sqrt((self.parentMine.x-self.x)**2 + (self.parentMine.y-self.y)**2)
                targetNodeDistance = np.sqrt((self.parentMine.x-node.x)**2 + (self.parentMine.y - node.y))
                currentNodeAngle = np.arccos(self.x-self.parentMine.x/currentNodeDistance)
                targetNodeAngle = np.arccos(node.x-node.parentMine.x/targetNodeDistance)

                distance = np.abs((self.parentMine.getRadius()*currentNodeAngle)-(node.parentMine.getRadius()*targetNodeAngle))

            else: # Nodes are on seperate mines
                distance = np.sqrt((self.x-node.x)**2+(self.y-node.y)**2)
        return distance

    # Get status of node in a list of strings: ["terminated"/"exists", "connected"/"disconnected"]
    def status(self) -> list:
        result = []
        if self.terminated:
            result.append("terminated")
        else:
            result.append("exists")
        
        if self.connected:
            result.append("connected")
        else:
            result.append("disconnected")
        result.append("referenced " + str(getrefcount(self)) + " times")
        return result
    


    # Establishes a connection between nodes
    def connectNode(self,node:"Node") -> None:
        self.connectedNodes.append(node)
        node.connectedNodes.append(self)
        self.connected = True
        node.connected = True
        if node.parentMine not in self.parentMine.connectedMines and self.parentMine not in node.parentMine.connectedMines:
            self.parentMine.connectedMines.append(node.parentMine)
            node.parentMine.connectedMines.append(self.parentMine)
    
    # Check if node is within another mines' radius
    def validateNode(self):
        for mine in Mine.mines:
            if mine != self.parentMine:
                if (mine.x - radius <= self.x <= mine.x + radius and
                    mine.y - radius <= self.y <= mine.y + radius):
                    self.terminated = True
                    self.connectedNodes[0].terminated = True


    # Get type of path to a node -> ["line"/"arc", "established"/"unestablished","bitangent"/"normal"]
    def getPathType(self,node:'Node') -> list:
        types = []

        # Connection based on parent mines
        if node.parentMine != self.parentMine:
            types.append("line")
        elif node.parentMine == self.parentMine:
            types.append("arc")
        
        # Connection exists 
        if node in self.connectedNodes:
            types.append("established")
        elif node not in self.connectedNodes:
            types.append("unestablished")
        
        # Connection follows bitangency, such that moving from node to node has no sharp/perpendicular turns
        if (self.type == "internal primary" and node.type == "internal primary" or
            self.type == "external primary" and node.type == "external secondary" or
            self.type == "internal secondary" and node.type == "internal secondary" or
            self.type == "external secondary" and node.type == "external primary"):
            types.append("bitangent")
        else:
            types.append("normal")
        
        return types

    def getPos(self) -> float:
        return (round(float(self.x),3),round(float(self.y),3))

    def getTargetMine(self) -> Mine:
        return self.__targetMine
    def __str__(self):
        return self.name
    def __repr__(self):
        return self.__str__()


numMines = 50
radius = 16
debug = False
xMin = -numMines*radius*0.45
xMax = numMines*radius*0.45
yMin = -numMines*radius*0.45
yMax = numMines*radius*0.45
if not debug:
    field = Field(xMin,xMax,yMin,yMax)
else:
    field = Field(-200,200,-300,300)
genXMin = -radius*(numMines//5)
genXMax = radius*(numMines//5)
genYMin = -radius*(numMines//5)
genYMax = radius*(numMines//5)
position = [0,0] 
mineGenTolerance = 0*radius
# Mine generation
if not debug:
    for num in range(numMines):
        while True: # To make sure generated mines arent clipping off the edges of the field
            position[0], position[1] = random.randint(genXMin,genXMax+1),random.randint(genYMin,genYMax+1)
            invalidPosition = False
            for mine in Mine.mines:
                if (mine.getPos()[0] - mineGenTolerance <= position[0] <= mine.getPos()[0] + mineGenTolerance) and (mine.getPos()[1] - mineGenTolerance <= position[1] <= mine.getPos()[1] + mineGenTolerance):
                    invalidPosition = True
                    break
            if invalidPosition:
                continue
            if position[0] <= xMin + radius or position[0] >= xMax - radius or position[1] <= yMin + radius or position[1] >= yMax - radius:
                continue
            break
        field.addMine(position[0],position[1],radius)
        print(num)
    
    # for node in Node.nodes:
    #     print(node.getReferences())
    
    ## Example Concept for a single start point end point(s)
    # Start
    # field.addMine(-150,-300,radius,'green')
    # End Points
    # field.addMine(-300,300,radius,'black')
    # field.addMine(-250,300,radius,'black')
    # field.addMine(-200,300,radius,'black')
    # field.addMine(-150,300,radius,'black')
    # field.addMine(-50,300,radius,'black')
    # field.addMine(-100,300,radius,'black')
    # field.addMine(0,2500,radius,'black')#######
    # field.addMine(50,300,radius,'black')
    # field.addMine(100,300,radius,'black')
    # field.addMine(150,300,radius,'black')
    # field.addMine(200,300,radius,'black')
    # field.addMine(250,300,radius,'black')
    # field.addMine(300,300,radius,'black')
# for pair in combinations(Node.nodes,2):
#     connections = pair[0].getPathType(pair[1])
#     print(connections)
if debug:

    # Test aligned mines
    field.addMine(0,0,radius)
    field.addMine(-150,0,radius)
    field.addMine(150,0,radius)
    field.addMine(0,-150,radius)
    field.addMine(0,150,radius)
    field.addMine(50,50,radius)
    field.addMine(-50,50,radius)
    field.addMine(-50,-50,radius)
    field.addMine(50,-50,radius)

    # Hypothetical starting nodes as a mine
    field.addMine(0,-250,radius)

    # Test overlapping aligned mines
    field.addMine(-150,250,radius) # Middle
    field.addMine(-125,250,radius)
    field.addMine(-100,250,radius)
    field.addMine(-75,250,radius)
    field.addMine(-50,250,radius)
    field.addMine(-25,250,radius)
    field.addMine(0,250,radius)
    field.addMine(25,250,radius)
    field.addMine(50,250,radius)
    field.addMine(75,250,radius)
    field.addMine(100,250,radius)
    field.addMine(125,250,radius)
    field.addMine(150,250,radius)

    field.addMine(-125,275,radius) # Top
    field.addMine(-100,275,radius)
    field.addMine(-75,275,radius)
    field.addMine(-50,275,radius)
    field.addMine(-25,275,radius)
    field.addMine(0,275,radius)
    field.addMine(25,275,radius)
    field.addMine(50,275,radius)
    field.addMine(75,275,radius)
    field.addMine(100,275,radius)
    field.addMine(125,275,radius)
    
    field.addMine(-125,225,radius) # Bottom
    field.addMine(-100,225,radius)
    field.addMine(-75,225,radius)
    field.addMine(-50,225,radius)
    field.addMine(-25,225,radius)
    field.addMine(0,225,radius)
    field.addMine(25,225,radius)
    field.addMine(50,225,radius)
    field.addMine(75,225,radius)
    field.addMine(100,225,radius)
    field.addMine(125,225,radius)


    for mine in Mine.mines:
        print(mine,'connected to',','.join(m.__str__() for m in mine.connectedMines))
    print(Node.nodes)
    for node in Node.nodes:
        print()

field.plotField()
"""
TODO:
X=Done
- = Todo
 X Arc lengths
 - Find a way to set a viable and efficient end/start point
      - Overload Node __init__ to allow for a lone node as a start node
 X Terminate nodes if the nodes themselves are created within another mine
 - Establish a list of paths between connected Nodes
 - Terminate internal bitangents unless external bitangents are intersecting ???
 - Combine all node lists into one


"""
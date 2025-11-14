import matplotlib.pyplot as pyplot
import numpy as np
import random
from itertools import combinations
import time
"""
When using the attributes/methods, refer to the object unless intentionally accessing a class variable.

KEY MINE ATTRIBUTES/METHODS:
 - Mine.getPos() -> (x,y) position on field of the Mine
 - Mine.getNodes() -> [Node,Node,...] A list of all mine's child Nodes

KEY NODE ATTRIBUTES/METHODS:

 - Node.terminated -> True/False (Very IMPORTANT, this node should not be counted if terminated is True)

 - Node.type() -> "internal/external primary/secondary" (Either internal OR external AND primary OR secondary)
 - Node.parentMine -> Mine object that the node is on
 - Node.connectedNodes -> [Node,Node,...] A list of all nodes, excluding the Node itself
 - Node.getPathType(Node) -> ["line"/"arc", "established"/"unestablished","bitangent"/"normal"] 
                              A list of strings describing the kind of connection each node would be connected
 - Node.getPos() -> (x,y) position on field of the Node
 - Node.distanceFromNode(Node) -> distance (Simple distance between nodes)

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
    
    def addMine(self,centerX,centerY,radius,color:str=''):
        newMine = Mine(centerX,centerY,radius,color=color)
        self.mines.append(newMine)
        mineCombo = [[newMine, mine] for mine in self.mines[:-1]]
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
            connectedNode = node.connectedNode

            if not node.terminated and not connectedNode.terminated:
                x1 = float(node.x)
                y1 = float(node.y)
                x2 = float(connectedNode.x)
                y2 = float(connectedNode.y)

                for mine in self.mines:
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
        # Check all other nodes Excluding newly created nodes if they intersect newly created Mine
        for node in [n for n in Node.nodes if n not in newMine.nodes]:
            connectedNode = node.connectedNode
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
        
        # Brute force; each node has a list of every node created, excluding itself. 
        # Significantly slows down this algorithm though
        # for node in Node.nodes:
        #     node.connectedNodes.extend([n for n in Node.nodes if n not in [node,node.connectedNode]]`

        
    def plotField(self):
        plt = pyplot
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        plt.xlim(self.xMin,self.xMax)
        plt.ylim(self.yMin,self.yMax)
        
        # Create a list of circles representing mines, centered to their correlated mine's center

        circles = [plt.Circle(self.mines[i].getPos(),self.mines[i].getRadius(),color=self.mines[i].color) for i in range(len(self.mines))]
        
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
        # Node storage
        self.nodes = []
        self.connectedMines = []
        Mine.mines.append(self)
        
        
    def getPos(self):
        return self.x,self.y
    def getRadius(self):
        return self.radius
    def getNodes(self,type:str=''):
        if type == None or type == '':
            return self.nodes
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
        Node.nodes.append(node)
        self.nodes.append(node)
        self.nodes.append(node)
        return node
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
        self.connectedNodes = []
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

    def status(self) -> list:
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

    def connectNode(self,node:"Node") -> None:
        self.connectedNode = node
        node.connectedNode = self
        self.connected = True
        node.connected = True
        if node.parentMine not in self.parentMine.connectedMines and self.parentMine not in node.parentMine.connectedMines:
            self.parentMine.connectedMines.append(node.parentMine)
            node.parentMine.connectedMines.append(self.parentMine)
        return node

    # Get type of path to a node -> ["line"/"arc", "established"/"unestablished","bitangent"/"normal"]
    def getPathType(self,node:'Node') -> list:
        types = []

        # Connection based on parent mines
        if node.parentMine != self.parentMine:
            types.append("line")
        elif node.parentMine == self.parentMine:
            types.append("arc")
        
        # Connection exists 
        if self.connectedNode == node:
            types.append("established")
        elif self.connectedNode != node:
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


numMines = 100
xMin = -300
xMax = 300
yMin = -300
yMax = 300
radius = 16
field = Field(xMin,xMax,yMin,yMax)
debug = False
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
            for mine in field.mines:
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
    field.addMine(0,0,radius)
    field.addMine(-150,0,radius)
    field.addMine(150,0,radius)
    field.addMine(0,-150,radius)
    field.addMine(0,150,radius)
    field.addMine(50,50,radius)
    field.addMine(-50,50,radius)
    field.addMine(-50,-50,radius)
    field.addMine(50,-50,radius)

    field.addMine(0,-250,radius)

    for mine in field.mines:
        print(mine,'connected to',','.join(m.__str__() for m in mine.connectedMines))
    print(Node.nodes)

field.plotField()
"""
TODO:
X=Done
- = Todo
 X Arc lengths
 - Find a way to set a viable and efficient end point
 - Terminate nodes if the nodes themselves are created within another mine
 - ########## Terminate internal bitangents unless external bitangents are intersecting ##################

"""
import sys, os
sys.path.append(os.path.abspath(".."))
import matplotlib.patches as patches

import matplotlib.pyplot as pyplot
import numpy as np
import random
import time
from sys import getrefcount
import gc
from . import path_calculation
from enum import Enum

######## File: genNodesFromMines.py                                                                      ########
######## Purpose:                                                                                        ########
######## This file, given the center coordinates of mines and a to-be-determined radius, generates nodes ########
######## that connect to each other in a way that allows for traversal with minimal sharp turns.         ########

"""
When using the attributes/methods, refer to the object unless intentionally accessing a class variable.
KEY MINE ATTRIBUTES/METHODS:
 - Mine.getPos() -> (x,y) position on field of the Mine
 - Mine.getNodes() -> [Node,Node,...] A list of all mine's child Nodes, can be accessed directly with Mine.nodes
 - Mine.connectMineNodes() -> establishes node connections from the nodes on the current mine to the rest of the nodes.

KEY NODE ATTRIBUTES/METHODS:
 - Node(xPosition:float,yPosition:float,floating:bool,name:str) -> Constructor for general nodes, floating or not.
 - Node.connectNode(node:Node) -> returns Connection: Establishes connection between current and target Node
 - Node.getPos() -> gets position of node
 - Node.getTargetMine() -> returns Mine; gets the target mine
 - Node.getParentMine() -> returns Mine or None if floating; gets the parent mine
 
 - MineNode(parentMine:Mine, targetMine:Mine,internal:True/False,primary:True/False,connectedToFloating:True/False,floatingNode:Node,name:str)
    -> Constructor; not reconmended to construct manually, as the parameters heavily affect placements. Use Field to add Nodes.

 - MineNode.getPathType(Node) -> ["line"/"arc", "established"/"unestablished","bitangent"/"normal"] 
                              A list of strings describing the kind of connection each node has to another
 - MineNode.getPos() -> (x,y) position on field of the Node

KEY CONNECTION ATTRIBUTES/METHODS:
 - Connection(node1:Node,node2:Node) -> Constructor for pairing nodes to make a connection
 - Connection.addGraph() -> adds connection to field.nodeGraph
    (Connection objects are two-way and shared)
    {
    Node1: {Node2:ConnectionObj 1<->2, Node3:ConnectionObj 1<->3},
    Node2: {Node1:ConnectionObj 1<->2},
    Node3: {Node1:ConnectionObj 1<->3}
    }
 - Connection.deleteConnection() -> Removes itself from nodeGraph
 - Connection.validPath() -> Determines if any point along the connection collides with a mine.
 - Connection.mineCollision(self,mine) -> Determines if the connection collides with a specific mine.

KEY FIELD ATTRIBUTES/METHODS:
 = Class Variable: nodeGraph -> a dictionary containing Connection objects
 - Field.addMine(centerX,centerY,radius) -> Adds a mine to the field and creates and connects Nodes accordingly
 - Field.plotField(labeled:bool) -> Using matplotlib, plots the field. If labeled=True, labels the nodes.
                                    plot key: NID = Node ID; MID = Mine ID ; CID = [Parent Mine ID].[Node ID]
 - addFloatingNode(x,y): -> adds a node to the field that does not have a parent mine,
                            But does have a target mine.
 - placeStartNode(xVal,yVal) -> adds the starting Node(s). Must have at least one.
 - placeEndNodes(xMin,xMax,yVal,density) -> Places endpoint nodes along a certain y-value within
                                            the limits of xMin and xMax with a certain density
                                            or amount of nodes placed equidistantly.
---------------------------------------------------------------------

Use the Field class to generate mines and their nodes.
Unless you are optimizing or working on this program/code/algorithm, 
I suggest you create mines and nodes via the Field class.
    Field(xMin,xMax,yMin,yMax)
    Field Setup:
     - xMin,xMax : The fields x axis range
     - yMin, yMax : The fields y axis range
Ex: A basic setup of 3 Mines with start and end nodes.
    field = Field(-100,150,-100,150)
    field.addMine(0,0,20)
    field.addMine(-30,0,20)
    field.addMine(30,25,20)
    field.placeStartNode(0,-10) 
    field.placeEndNodes(-100,100,250,4)
    field.plotField()

Complex example moved to test_cases.py
"""
#Simple class which is used in the nodegraph and holds informatation about distance path and type (straight/arc-ed)
#Moved some useful connection functions here as well.
class seg(Enum):
    ARC=1
    LINE=2

class Connection:
    field:'Field'=None #must be initialized on startup
    def __init__(self,node1: 'Node',node2: 'Node'):
        
        self.node1=node1
        self.node2=node2
        
        if(node1.parentMine != node2.parentMine or node1.floating or node2.floating):
            self.connectionType=seg.LINE
        else:
            self.connectionType=seg.ARC

        if node1.parentMine and node2.parentMine:
            
            if(node1.parentMine != node2.parentMine):
                self.connectionType=seg.LINE
            else:
                self.connectionType=seg.ARC
        else:
            self.connectionType = seg.LINE
        self.distance=self.updateDistance()

        #checking for a valid path and updating the graph must be done manually
        
        # DISTANCE
    def updateDistance(self):
        distance = 0.0
        
        if self.connectionType==seg.ARC: # Nodes are on the same mine
            angleTheta=abs(self.node1.angle-self.node2.angle)
            
            mineRadius=self.node1.parentMine.radius

            distance = angleTheta*mineRadius

        else: # Nodes are on seperate mines
            distance = np.sqrt((self.node1.x-self.node2.x)**2+(self.node1.y-self.node2.y)**2)
            distance=float(distance)
        return distance
    

    #Graph organization:
    """
    (Connection objects are two-way and shared)
    {
    Node1: {Node2:ConnectionObj 1<->2, Node3:ConnectionObj 1<->3},
    Node2: {Node1:ConnectionObj 1<->2},
    Node3: {Node1:ConnectionObj 1<->3}
    }
    """
    def addGraph(self):
        if(self.node1 not in self.field.nodeGraph):
            self.field.nodeGraph.update({self.node1:{self.node2:self.distance}})
        elif self.node2 not in self.field.nodeGraph[self.node1]:
            self.field.nodeGraph[self.node1].update({self.node2:self.distance})
        # else: # Im not sure why this would be considered broken
            # print("node1 is in field.nodeGraph AND node2 in node1's nodeGraph")
            # print("Something Broke")


        if(self.node2 not in self.field.nodeGraph): #Needed for its first connection: When a node is made, it's key is automatically added to nodeGraph with a none value.
            self.field.nodeGraph.update({self.node2:{self.node1:self.distance}}) #Must use = to get rid of the none value
        elif self.node1 not in self.field.nodeGraph[self.node2]:
            self.field.nodeGraph[self.node2].update({self.node1:self.distance})
        # else: # Im not sure why this would be considered broken
        #     print("node2 is in field.nodeGraph AND node1 in node2's nodeGraph")
        #     print("Something Broke")

    def deleteConnection(self):

        if self.node1 in self.field.nodeGraph: 
            if self.node2 in self.field.nodeGraph[self.node1]:
                del self.field.nodeGraph[self.node1][self.node2]
            if len(self.field.nodeGraph[self.node1])==0:
                self.node1.deleteNode()
        else:
            self.node1.deleteNode()

        if self.node2 in self.field.nodeGraph: 
            if self.node1 in self.field.nodeGraph[self.node2]:
                del self.field.nodeGraph[self.node2][self.node1]
            if len(self.field.nodeGraph[self.node2])==0:
                self.node2.deleteNode()
        else:
            self.node2.deleteNode()
         
    #Checks if a newly created path is valid, checks all mines for collisions
    def validPath(self):
        if(self.node1==self.node2):
            return False
        x1 = float(self.node1.x)
        y1 = float(self.node1.y)
        x2 = float(self.node2.x)
        y2 = float(self.node2.y)
        if self.connectionType == seg.LINE:
            for mine in Connection.field.mines:
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
                    if distanceFromMine < mine.radius:
                        return False

                
                # Check if node is in mine
                n1distance = np.sqrt(((x1-x3)**2) + ((y1-y3)**2))
                n2distance = np.sqrt(((x2-x3)**2) + ((y2-y3)**2))

                if self.node1.parentMine != mine:
                    if n1distance <= mine.radius:
                        return False
                if self.node2.parentMine != mine:
                    if n2distance <= mine.radius:
                        return False
        elif self.connectionType == seg.ARC:
            parentMine = self.node1.parentMine
            validEdge=True
            for mine in parentMine.overlappingMines:
                # Other than being None, there should only be 2 values
                intersectionPoints,intersectionAngle,offsetAngle = self.generateIntersectionPoints(parentMine,mine)
                
                if intersectionPoints != None:
                    validEdge = validEdge and self.validHuggingEdge(intersectionAngle,offsetAngle)
                
                else:
                    print("Something went really wrong with midpoint & intersectionpoints")
            return validEdge
        
        return True

    #checks if a path collides with a specific mine
    def mineCollision(self,mine) -> bool:
        if(self.node1==self.node2):
            return True
        
        x1 = float(self.node1.x)
        y1 = float(self.node1.y)
        x2 = float(self.node2.x)
        y2 = float(self.node2.y)
        if self.connectionType == seg.LINE:
            
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
                if distanceFromMine < mine.radius:
                    return True

            
            # Check if node is in mine
            n1distance = np.sqrt(((x1-x3)**2) + ((y1-y3)**2))
            n2distance = np.sqrt(((x2-x3)**2) + ((y2-y3)**2))

            if self.node1.parentMine != mine:
                if n1distance <= mine.radius:
                    return True
            if self.node2.parentMine != mine:
                if n2distance <= mine.radius:
                    return True
        elif self.connectionType == seg.ARC:
            parentMine = self.node1.parentMine
            
            # Other than being None, there should only be 2 values
            intersectionPoints,intersectionAngle,offsetAngle = self.generateIntersectionPoints(parentMine,mine)

            if intersectionPoints != None:
                validEdge = self.validHuggingEdge(intersectionAngle,offsetAngle)
                return not(validEdge)
        
        return False
    
    def validHuggingEdge(self,intersectionAngle,offsetAngle) -> bool:
        node1 = self.node1
        node2 = self.node2
        firstNodeAngle=node1.angle
        secondNodeAngle=node2.angle
        #WE WILL ALWAYS ASSUME WE ARE MOVING COUNTER CLOCKWISE with node1 first and node2 second
        #^^^^^^^^^^^^^^^^^^^^^^
        
        intersectionPointAngle1,intersectionPointAngle2= intersectionAngle+offsetAngle,intersectionAngle-offsetAngle

        intersectionPointAngle1%=np.pi*2
        intersectionPointAngle2%=np.pi*2

        if(intersectionPointAngle1<0):
            intersectionPointAngle1+=np.pi*2
        if(intersectionPointAngle2<0):
            intersectionPointAngle2+=np.pi*2

        # print(f"interscetion angle {intersectionAngle}")
        # print(f"offset angle {offsetAngle}")
        # print(f"firstNode ANgle {firstNodeAngle}")
        # print(f"second node ANgle {secondNodeAngle}")

        #First if handles case where the hugging edge travels over angle=0.
        if(firstNodeAngle>secondNodeAngle):
            if(intersectionPointAngle1>firstNodeAngle and intersectionPointAngle1<secondNodeAngle+np.pi*2):
                return False
            elif(intersectionPointAngle1<secondNodeAngle and intersectionPointAngle1>firstNodeAngle-np.pi*2):
            
                return False
        else:
            if(firstNodeAngle<intersectionPointAngle1<secondNodeAngle):
                return False
            if(firstNodeAngle<intersectionPointAngle2<secondNodeAngle):
                return False
        return True
    
    """Generating the points where mines intersect"""
    @staticmethod # Used for logic elsewhere in this class, but does not need stuff from an instance
    def generateIntersectionPoints(mine1:"Mine",mine2:"Mine") -> list[float]:
        distance : float = np.sqrt((mine1.x-mine2.x)**2 + (mine1.y-mine2.y)**2)
        # Fraction of the area of each mine that is not overlapping
        radicalLineDistance: float = ((mine1.radius**2 - mine2.radius**2 + distance**2))/(2*distance)
        
       
        # Plus and minus this angle to get the angle at which the circles overlap
        offsetAngle = np.arccos(radicalLineDistance/mine1.radius)
        intersectionAngle=np.atan2(mine2.y-mine1.y,mine2.x-mine1.x)
        if(intersectionAngle<0):
            intersectionAngle+=np.pi*2
        
        intersectP1 = [mine1.radius * np.cos(intersectionAngle + offsetAngle) + mine1.x, mine1.radius * np.sin(intersectionAngle + offsetAngle) + mine1.y]
        intersectP2 = [mine1.radius * np.cos(-intersectionAngle + offsetAngle) + mine1.x, mine1.radius * np.sin(-intersectionAngle + offsetAngle) + mine1.y]
        return (intersectP1,intersectP2),intersectionAngle,offsetAngle

    def __str__(self):
        return f"{self.node1} <-> {self.node2}"
    def __repr__(self):
        return self.__str__()

# Field generates nodes off of mines, generates mines too
class Field:
    mines = []
    debugPoints = [] # purely for debuging and testing, field will plot these points
    def __init__(self,xMin,xMax,yMin,yMax):
        self.nodeGraph={}
        self.xMin = xMin
        self.yMin = yMin
        self.xMax = xMax
        self.yMax = yMax
        self.mines = []
        Connection.field=self

    # This type of node will not have a parent mine, primarily used for start/end points
    def addFloatingNode(self,x:int,y:int) ->'Node':
        fNode = Node(x,y,True) # Floating Node
        
        for mine in Connection.field.mines:
            mineNodePrimary = MineNode(parentMine=mine,floatingNode=fNode,primary=True,connectedToFloating=True)
            mineNodeSecondary = MineNode(parentMine=mine, floatingNode=fNode, primary=False,connectedToFloating=True)
            mine.addNode(mineNodePrimary)
            mine.addNode(mineNodeSecondary)
            mineNodePrimary.connectNode(fNode)
            mineNodeSecondary.connectNode(fNode)
        
        if fNode in self.nodeGraph:
            if len(self.nodeGraph[fNode])==0:
                del self.nodeGraph[fNode]
        return fNode
    
    #Due to the current node stucture, right now this only modifies the nodeGraph
    def placeStartNode(self,xVal:int ,yVal:int ) -> 'Node':
        start = self.addFloatingNode(xVal,yVal)
        return start    
    # Places density amount of end nodes equidistance along the y coordinate and between xMin and xMax
    def placeEndNodes(self, yVal: int, density: int):
        returnList=[]
        if density > 1:
            xVals = [self.xMin + (i * ((self.xMax-self.xMin)/density-1)) for i in range(density)]
            for x in xVals:
                returnList.append(self.addFloatingNode(x,yVal))
        else:
            returnList.append(self.addFloatingNode(0,yVal))
        return returnList

    def addMine(self,centerX,centerY,radius,color:str=''):
        newMine = Mine(centerX,centerY,radius,color=color)
        self.mines.append(newMine)
        mineCombo = [[newMine, mine] for mine in self.mines[:-1]]
        for pair in mineCombo:
            mine = pair[0]
            target = pair[1]
            
            # Best not to mess with how nodes are placed on the mine, 
            # the combinations of the booleans might be confusing

            # Mine internal nodes
            mineInternPrimary = MineNode(mine,target,True,True)
            mineInternSecond = MineNode(mine,target,True,False)
            mine.addNode(mineInternPrimary)
            mine.addNode(mineInternSecond)

            # Mine External nodes
            mineExternPrimary = MineNode(mine,target,False,True)
            mineExternSecond = MineNode(mine,target,False,False)
            mine.addNode(mineExternPrimary)
            mine.addNode(mineExternSecond)
            
            # target internal nodes
            targetInternPrimary = MineNode(target,mine,True,False)
            targetInternSecond = MineNode(target,mine,True,True)
            target.addNode(targetInternPrimary)
            target.addNode(targetInternSecond)
            # target external nodes
            targetExternPrimary = MineNode(target,mine,False,False)
            targetExternSecond = MineNode(target,mine,False,True)
            target.addNode(targetExternPrimary)
            target.addNode(targetExternSecond)
            
            # Connect Nodes
            #connectNode will check if it is a valid connection and add it to the nodeGraph it is.
            mineInternPrimary.connectNode(targetInternSecond)
            mineInternSecond.connectNode(targetInternPrimary)

            mineExternPrimary.connectNode(targetExternPrimary)
            mineExternSecond.connectNode(targetExternSecond)
            #mine.connectMineNodes()
            #target.connectMineNodes()
            mine.updateOverlap(target)


        
        shallowCopy=self.nodeGraph.copy()
        # Check if any of the other node connections pass through the newly created mine.
        for node1 in [n for n in shallowCopy.keys() if n not in newMine.nodes]:
           deepCopy=shallowCopy[node1].copy()
           for node2 in deepCopy:
                connection=Connection(node1,node2)
                if(connection.mineCollision(newMine)):
                    connection.deleteConnection()

    
    # Purely for debugging  
    def plotField(self,labeled:bool=False,path=[],title:str="Mines and Potential Paths",xlabel:str="") -> None:
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
        for mine in Mine.mines:
            if labeled:
                vertalignment = ['top','bottom','baseline','center_baseline']
                horzalignment = ['left','right','center']
                plt.text(mine.x,mine.y,str(mine),horizontalalignment=random.choice(horzalignment),verticalalignment=random.choice(vertalignment),bbox=dict(facecolor=(0.5,0.5,0.5),alpha=0.3,linewidth=0))
            plt.plot(mine.x,mine.y,"x",color=(1,1,1))
        nodeSymbol = '' # Empty string makes either lines or invisible points; otherwise points are displayed using the symbol
        print("Start plotting, will not affect node generation")
        for node in self.nodeGraph.keys():
            if labeled or node.labeled:
                vertalignment = ['top','bottom','baseline','center_baseline']
                horzalignment = ['left','right','center']
                plt.text(node.x, node.y, str(node),horizontalalignment=random.choice(horzalignment),verticalalignment=random.choice(vertalignment),c=(0.0,0.0,0.0))

            if not node.plotted:
                for connectedNode in self.nodeGraph[node].keys():
                    # If it is an arc connection, same parent mines, then draw a curve
                    if(connectedNode.parentMine==node.parentMine and node.parentMine!=None):
                        plt.plot([node.x,connectedNode.x],[node.y,connectedNode.y],nodeSymbol)
                    else:
                        # Otherwise, draw a line
                        # pass
                        try:
                            
                            plt.plot([node.x,connectedNode.x],[node.y,connectedNode.y],nodeSymbol)
                        except AttributeError:
                            plt.plot([node.x],[node.y],nodeSymbol)
        
        # If a path is passed in, display the path as a black line
        if len(path) > 0:
            if len(xlabel) <= 0:
                xlabel += "Black = A* path"
            for i, node in enumerate(path):
                if (i < len(path)-1):
                    nextNode = path[i+1]
                    plt.plot([node.x,nextNode.x],[node.y,nextNode.y],color=(0,0,0))
                        
        if len(Field.debugPoints) > 0: # Points that are plotted for debugging only
            print("Plotting debug points")
            for point in Field.debugPoints:
                plt.plot(point[0],point[1],"o",color=(0,0,0))
        
        print("Done plotting")
        print("Displaying field...")
        
        plt.title(title)
        plt.xlabel(xlabel)
        plt.show()

    # Run this to remove nodes that have no associated connection, ie, {node: None}
    def cleanNodeGraph(self):
        if self.nodeGraph != None:
            for node in self.nodeGraph.copy():
                if self.nodeGraph[node] == None:
                    del self.nodeGraph[node]
            return self.nodeGraph
        else:
            print("Node graph is empty")

    def increaseRadius(self,step): 
        shallowCopy=self.nodeGraph.copy()
        #Delete all arc connections
        for node1 in shallowCopy.keys():
           deepCopy=shallowCopy[node1].copy()
           for node2 in deepCopy:
                connection=Connection(node1,node2)
                if(connection.connectionType==seg.ARC):
                    connection.deleteConnection()
        Mine.radius+=step
        for mine in self.mines:
            mine.radius+=step
        #Avoids the case where two mines will/would overlap, but one's radius hasn't been increased yet
        for mine in self.mines:
            for target in self.mines:
                    mine.updateOverlap(target)


        

        shallowCopy=self.nodeGraph.copy()
        for node1 in shallowCopy:
            if(node1.parentMine!=None):
                
                node1.calculateAndAssignPosition()
        #Delete all invalid connections
        for node1 in shallowCopy:
           deepCopy=shallowCopy[node1].copy()
           for node2 in deepCopy:
                oldConnection=Connection(node1,node2)
                if(not(oldConnection.validPath())):
                    # print(f"deleting {oldConnection}")
                    oldConnection.deleteConnection()

        for mine in self.mines:
            mine.connectMineNodes()
        

"""MATH STUFF"""
# Mine class keeps track of mine position and radii      
class Mine:
    numMines = 0
    mines = []
    radius = None
    def __init__(self,centerX,centerY,radius,color:str='',name:str=''):
        
        Mine.radius = radius
            
        Mine.numMines += 1
        self.number=Mine.numMines
        if len(name) > 0:
            self.name = name
        else:
            self.name = f'MID: ' + str(self.numMines)
        
        if len(color) > 0:
            self.color = color
        else:
            
            self.color = random.randint(20,80)/100,random.randint(20,80)/100,random.randint(20,80)/100
        self.x, self.y, self.radius = np.round(centerX,2) , np.round(centerY,2), np.round(radius,2)
        # Node storage
        self.nodes = [] 
        self.connectedMines = []
        self.overlappingMines = []
        Mine.mines.append(self)
        
    def getPos(self):
        return self.x,self.y
    def getRadius(self):
        return Mine.radius
    def getNodes(self):
        return self.nodes    

    def addNode(self,node:"Node"):
        #Node.nodes.append(node)

        self.nodes.append(node)
        #self.nodes.append(node) 
        return node
    def removeNode(self,node):
        self.nodes.remove(node)
    def connectMineNodes(self):
        
        sortedNodes = sorted(self.nodes, key=lambda node: node.angle)
        if len(sortedNodes)==0:
            return 0

       
        for i in range(len(sortedNodes)-1):
           
            arcConnection = Connection(sortedNodes[i],sortedNodes[i+1])
            if(arcConnection.validPath()):
                arcConnection.addGraph()
        arcConnection = Connection(sortedNodes[len(sortedNodes)-1],sortedNodes[0])
        if(arcConnection.validPath()):
            arcConnection.addGraph()
       
        #TODO: validate that path doesn't run through another mine. Validation isn't setup yet
        
        return getrefcount(self)

    #Checks if target and self overlap. If they do update both's overlappingMines list.
    def updateOverlap(self,target : 'Mine'):
        # Add to each mines' overlapping mines list if they do overlap
        if target not in self.overlappingMines and self not in target.overlappingMines:
            distanceThreshold = 2*Mine.radius
            distance = np.sqrt((self.x-target.x)**2 + (self.y-target.y)**2)
            if distance < distanceThreshold:
                self.overlappingMines.append(target)
                target.overlappingMines.append(self)
    def __str__(self):
        return self.name
    def __repr__(self):
        return self.__str__()

# Node class keeps track of node positions
class Node:
    nodeNum = 0
    connectionList = []
    
    def __init__(self, xPosition: float, yPosition:float,floating:bool,angle:float=0,name:str="",labeled:bool=False):
        Node.nodeNum += 1
        if len(name) < 1:
            self.name = "NID: "+str(Node.nodeNum)
        else:
            self.name = name 
        self.type = type
        self.labeled = labeled # Purely for debugging and visually isolating nodes
        self.x = xPosition
        self.y = yPosition
        self.plotted = False # To prevent hopefully duplicate plotting
        #self.nodeGraph.update({self:None})
        self.parentMine=None
        self.floating=floating
        self.angle = angle # will stay 0 if node doesnt have an angle, AKA it is floating


   
    # Establishes a connection between nodes
    # Does not add it to the nodegraph yet however
    def connectNode(self,node:"Node") -> Connection:
        if(self==node):
            raise TypeError("Same nodes")
        nodeConnection=Connection(self,node) #connection initialization 
        if(nodeConnection.validPath()):
            nodeConnection.addGraph()
        else:
            nodeConnection.deleteConnection()
        #self.connected = True
        #node.connected = True
        """
        if node.parentMine not in self.parentMine.connectedMines and self.parentMine not in node.parentMine.connectedMines:
            self.parentMine.connectedMines.append(node.parentMine)
            node.parentMine.connectedMines.append(self.parentMine)

        """
        return nodeConnection
    def deleteNode(self):
        if self.parentMine != None:
            self.parentMine.removeNode(self)
        if self in Connection.field.nodeGraph:
            del Connection.field.nodeGraph[self]
    def getPos(self) -> float:
        return (round(float(self.x),3),round(float(self.y),3))

    def getTargetMine(self) -> Mine:
        return self.__targetMine
    def getParentMine(self) -> Mine:
        return self.parentMine
    def __str__(self):
        return self.name
    def __repr__(self):
        return self.__str__()
    def __gt__(self, node1:'Node'):
        return self.y>node1.y


class MineNode(Node):
    
    def __init__(self,parentMine:"Mine"=None,targetMine:"Mine"=None,internal:bool=True,primary:bool=True,connectedToFloating:bool=False,floatingNode:"Node"=None,name:str=''):
        Node.nodeNum += 1
        if len(name) < 1:
            self.name = "FNID: "+ str(Node.nodeNum)
        else:
            self.name = name 
        self.parentMine = parentMine
        self.x = 0.0
        self.y = 0.0
        self.angle=0.0
        self.connected = False
        self.plotted = False # To prevent hopefully duplicate plotting
        self.targetMine = targetMine
        self.terminate = False # If node would be generated illegally, mark for termination/ignoring
        self.internal=internal
        self.primary=primary
        # These should not be used if connectedToFloating is False
        self.connectedToFloating = connectedToFloating
        self.floatingNode = floatingNode

        self.calculateAndAssignPosition()
        
        super().__init__(self.x,self.y,False,angle=self.angle,name=self.name)
        self.parentMine = parentMine #VERY NECESSARY DO NOT REMOVE
        if len(name) < 1:
            self.name = "CID:"+str(parentMine.number)+"."+str(Node.nodeNum)
        else:
            self.name = name
    
    def calculateAndAssignPosition(self):
        
        if not self.connectedToFloating:  # MineNode will have slightly different variables depending on this

            # categorize nodes
            if self.internal and self.primary:
                self.type = 'internal primary'
            elif self.internal and not self.primary:
                self.type = 'internal secondary'
            elif not self.internal and self.primary:
                self.type = 'external primary'
            elif not self.internal and not self.primary:
                self.type = 'external secondary'

            d = np.sqrt((self.parentMine.x-self.targetMine.x)**2+(self.parentMine.y-self.targetMine.y)**2) # Algabraic Distance Formula

            # Primary node is the first node where it is placed 
             # (typically towards the top of the circle)
            # Create Angle Offset(relative to target mine). It changes slightly depending on mines' positions
            # Formula is: arccos(x1-x2)/d+pi
        
            if self.parentMine.y > self.targetMine.y:
                offsetAngle =  np.arccos(np.clip((self.parentMine.x-self.targetMine.x)/d,-1,1))+np.pi
            elif self.parentMine.y < self.targetMine.y:
                offsetAngle = -np.arccos(np.clip((self.parentMine.x-self.targetMine.x)/d,-1,1))+np.pi
            elif self.parentMine.y == self.targetMine.y:
                if self.parentMine.x < self.targetMine.x:
                    offsetAngle =  np.arccos(np.clip((self.parentMine.x-self.targetMine.x)/d,-1,1))+np.pi
                elif self.parentMine.x > self.targetMine.x:
                    offsetAngle = -np.arccos(np.clip((self.parentMine.x-self.targetMine.x)/d,-1,1))+np.pi
            
            # Offset Angle is the same for internal and external bitangents
            if self.internal:
                # Create internal angle
                internalArccosParameter = ((self.parentMine.radius)+(self.targetMine.radius))/d
                internalAngle = np.arccos(np.clip(internalArccosParameter,-1,1))
                
                if self.primary:
                    self.angle=internalAngle+offsetAngle
                    self.x = ((self.parentMine.radius) * np.cos(self.angle)) + self.parentMine.x
                    self.y = ((self.parentMine.radius) * np.sin(self.angle)) + self.parentMine.y
                else:
                    self.angle=internalAngle-(offsetAngle)
                    self.x = (self.parentMine.radius) * np.cos(self.angle) + self.parentMine.x
                    self.y = (self.parentMine.radius) * np.sin(self.angle+np.pi) + self.parentMine.y               
            else:
                # Create external angle
                externalArccosParameter = (self.parentMine.radius-self.targetMine.radius)/d
                externalAngle = np.arccos(np.clip(np.abs(externalArccosParameter),-1,1))

                if self.primary:
                    self.angle=externalAngle+offsetAngle
                    self.x = ((self.parentMine.radius) * np.cos(self.angle)) +self.parentMine.x
                    self.y = ((self.parentMine.radius) * np.sin(self.angle)) + self.parentMine.y
                else:
                    self.angle=externalAngle-offsetAngle+np.pi
                    self.x = (self.parentMine.radius) * np.cos(self.angle-np.pi) + self.parentMine.x
                    self.y = (self.parentMine.radius) * np.sin(self.angle) + self.parentMine.y
            
            

            self.x = round(self.x,3)
            self.y = round(self.y,3)

        else: # This assumes there is no targetMine, only a parentMine,primary, and floatingNode
            d = np.sqrt((self.parentMine.x-self.floatingNode.x)**2+(self.parentMine.y-self.floatingNode.y)**2) # Algabraic Distance Formula
            internalArccosParameter = ((self.parentMine.radius))/d
            internalAngle = np.arccos(np.clip(internalArccosParameter,-1,1))
            
            
            if self.parentMine.y > self.floatingNode.y:
                offsetAngle =  np.arccos(np.clip((self.parentMine.x-self.floatingNode.x)/d,-1,1))+np.pi
            elif self.parentMine.y < self.floatingNode.y:
                offsetAngle = -np.arccos(np.clip((self.parentMine.x-self.floatingNode.x)/d,-1,1))+np.pi
            elif self.parentMine.y == self.floatingNode.y:
                if self.parentMine.x < self.floatingNode.x:
                    offsetAngle =  np.arccos(np.clip((self.parentMine.x-self.floatingNode.x)/d,-1,1))+np.pi
                elif self.parentMine.x > self.floatingNode.x:
                    offsetAngle = -np.arccos(np.clip((self.parentMine.x-self.floatingNode.x)/d,-1,1))+np.pi

            if self.primary:
                self.angle=internalAngle+offsetAngle
                self.x = ((self.parentMine.radius) * np.cos(self.angle)) + self.parentMine.x
                self.y = ((self.parentMine.radius) * np.sin(self.angle)) + self.parentMine.y
            else:
                self.angle=internalAngle-(offsetAngle)
                self.x = (self.parentMine.radius) * np.cos(self.angle) + self.parentMine.x
                self.y = (self.parentMine.radius) * np.sin(self.angle+np.pi) + self.parentMine.y
        
        self.angle=(np.atan2(self.y-self.parentMine.y,self.x-self.parentMine.x)+2*np.pi)%(2*np.pi)
        # print(self)
        # print(self.angle)
        self.x = round(self.x,3)
        self.y = round(self.y,3)

    # Get type of path to a node -> ["line"/"arc", "established"/"unestablished","bitangent"/"normal"]
    def getPathType(self,node:'Node') -> list:
        types = []

        # Connection based on parent mines
        if node.parentMine != self.parentMine:
            types.append("line")
        elif node.parentMine == self.parentMine:
            types.append("arc")
        

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

    def __str__(self):
        return self.name
    def __repr__(self):
        return self.__str__()

## Moved debugging to testCases.py ##
"""
TODO:
X=Done
- = Todo
 X Arc lengths
 X Terminate nodes if the nodes themselves are created within another mine
 X Establish a list of paths between connected Nodes
 X Terminate internal bitangents unless external bitangents are intersecting
 X Combine all node lists into one
 X Generate Floating Nodes
 X Generate tangent mineNodes connecting to floating nodes
 -  Expanding mines
 X Hugging 
"""
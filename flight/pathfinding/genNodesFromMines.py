import sys, os
sys.path.append(os.path.abspath(".."))

import matplotlib.pyplot as pyplot
import numpy as np
import random
import time
from sys import getrefcount
import gc
from . import genPathFromNodes
from enum import Enum

######## Programmer(s): Jack Dabrowski, Harper Reinhardt                                                 ########
######## Date: mm/dd/yy                                                                                  ########
######## File: genNodesFromMines.py                                                                      ########
######## Purpose:                                                                                        ########
######## This file, given the center coordinates of mines and a to-be-determined radius, generates nodes ########
######## that connect to each other in a way that allows for traversal with minimal sharp turns.          ########

"""
When using the attributes/methods, refer to the object unless intentionally accessing a class variable.
KEY MINE ATTRIBUTES/METHODS:
 - Mine.getPos() -> (x,y) position on field of the Mine
 - Mine.getNodes() -> [Node,Node,...] A list of all mine's child Nodes, can be accessed directly with Mine.nodes
 - Mine.connectMineNodes() -> establishes node connections from the nodes on the current mine to the rest of the nodes.

KEY NODE ATTRIBUTES/METHODS:
 = Class Variable: nodeGraph -> a dictionary containing Connection objects
 - Node(xPosition:float,yPosition:float,floating:bool,name:str) -> Constructor for general nodes, floating or not.
 - Node.connectNode(node:Node) -> returns Connection: Establishes connection between current and target Node
 - Node.getPos() -> gets position of node
 - Node.getTargetMine() -> returns Mine; gets the target mine
 - Node.getParentMine() -> returns Mine or None if floating; gets the parent mien
 - MineNode(parentMine:Mine, targetMine:Mine,internal:True/False,primary:True/False,name:str)
    -> Constructor; not reconmended to construct manually, as the parameters heavily affect placements. Use Field to add Nodes.

 - MineNode.terminated -> True/False (Very IMPORTANT, this node should not be counted if terminated is True)
                    |-For some reason, trying to completely delete all references to a Node breaks stuff-|

 - MineNode.getPathType(Node) -> ["line"/"arc", "established"/"unestablished","bitangent"/"normal"] 
                              A list of strings describing the kind of connection each node has to another
 - MineNode.getPos() -> (x,y) position on field of the Node

KEY CONNECTION ATTRIBUTES/METHODS:
 - Connection(node1:Node,node2:Node) -> Constructor for pairing nodes to make a connection
 - Connection.addGraph() -> adds connection to Node.nodeGraph
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

For now, there is a more complex example at the bottom for testing. Will be removed or moved eventually.
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
            # print(distance)
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


        if(self.node1.nodeGraph[self.node1]==None): #Needed for its first connection: When a node is made, it's key is automatically added to nodeGraph with a none value.
            Node.nodeGraph[self.node1]={self.node2:self.distance} #Must use = to get rid of the none value
        elif self.node2 not in Node.nodeGraph[self.node1]:
            Node.nodeGraph[self.node1].update({self.node2:self.distance})

               
        
        if(Node.nodeGraph[self.node2]==None): #Needed for its first connection: When a node is made, it's key is automatically added to nodeGraph with a none value.
            Node.nodeGraph[self.node2]={self.node1:self.distance} #Must use = to get rid of the none value
        
        elif self.node1 not in Node.nodeGraph[self.node2]:
            Node.nodeGraph[self.node2].update({self.node1:self.distance})
        
    def deleteConnection(self):
        if Node.nodeGraph[self.node1]!=None and self.node2 in Node.nodeGraph[self.node1]:
            del Node.nodeGraph[self.node1][self.node2]

        if Node.nodeGraph[self.node2]!=None and self.node1 in Node.nodeGraph[self.node2]:
            del Node.nodeGraph[self.node2][self.node1]

        '''
        
        node.connected = True
        connectedNode.connected = True
        if connectedNode.parentMine not in node.parentMine.connectedMines and node.parentMine not in connectedNode.parentMine.connectedMines:
            node.parentMine.connectedMines.append(connectedNode.parentMine)
            connectedNode.parentMine.connectedMines.append(node.parentMine)
        '''
    #Checks if a newly created path is valid, checks all mines for collisions
    def validPath(self):
        if(self.node1==self.node2):
            return False
        if self.connectionType==seg.LINE:
            x1 = float(self.node1.x)
            y1 = float(self.node1.y)
            x2 = float(self.node2.x)
            y2 = float(self.node2.y)

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
            
        #This is kinda complicated, but we have to check the hugging edge. 
        if self.connectionType==seg.ARC:
            print("We haven't implemented this yet")
            #raise NotImplemented()
        return True

    #checks if a path collides with a specific mine
    def mineCollision(self,mine):
        x1 = float(self.node1.x)
        y1 = float(self.node1.y)
        x2 = float(self.node2.x)
        y2 = float(self.node2.y)
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
        return False


# Field generates nodes off of mines, generates mines too
class Field:
    mines = []
    def __init__(self,xMin,xMax,yMin,yMax):
        self.nodeGraph={}
        self.xMin = xMin
        self.yMin = yMin
        self.xMax = xMax
        self.yMax = yMax
        Connection.field=self

    # This type of node will not have a parent mine, primarily used for start/end points
    def addFloatingNode(self,x:int,y:int) ->'Node':
        fNode = Node(x,y,True) # Floating Node
        for node in Node.nodeGraph.keys():
            connect = Connection(fNode,node)
            if connect.validPath():
                connect.addGraph()
            else:
                connect.deleteConnection()
        return fNode
            
    #Due to the current node stucture, right now this only modifies the nodeGraph
    def placeStartNode(self,xVal:int ,yVal:int ) -> 'Node':
        return self.addFloatingNode(xVal,yVal)
    
    # Places density amount of end nodes equidistance along the y coordinate and between xMin and xMax
    def placeEndNodes(self,xMin:int,xMax:int, yVal: int, density: int):
        returnList=[]
        xVals = [x for x in range(xMin,xMax,xMax//(density//2))]
        for x in xVals:
            returnList.append(self.addFloatingNode(x,yVal))
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
            mineInternPrimary.connectNode(targetInternSecond)
            mineInternSecond.connectNode(targetInternPrimary)

            mineExternPrimary.connectNode(targetExternPrimary)
            mineExternSecond.connectNode(targetExternSecond)
           
           
        """Terminate pair of Nodes in certain conditions"""
        # Check newMine Nodes intersecting other mines
        for node in newMine.nodes:
            #print(node.connections[-1].validPath())
            if(node.connections[0].validPath()):
                node.connections[0].addGraph()
            
        # Check all other nodes Excluding newly created nodes if they intersect newly created Mine
        for node in [n for n in Node.nodeGraph.keys() if n not in newMine.nodes]:
            for connection in node.connections:
                if(connection.mineCollision(newMine)):
                    connection.deleteConnection()
        
    def plotField(self,labeled:bool=False):
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
                plt.text(mine.x,mine.y,str(mine),horizontalalignment='center',verticalalignment='center',bbox=dict(facecolor=(0.5,0.5,0.5),alpha=0.3,linewidth=0))

        # Plot the nodes
        nodeSymbol = '' # Empty string makes either lines or invisible points; otherwise points are displayed using the symbol
        print("Start plotting, will not affect node generation")
        for node in Node.nodeGraph.keys():
            if labeled:
                plt.text(node.x, node.y, str(node),horizontalalignment='center',verticalalignment='center',c=(0.0,0.0,0.0))

            if not node.plotted and not node.terminated:
                if Node.nodeGraph[node] != None: # On the chance a node does not have a connection, skip over the node
                    for connectedNode in Node.nodeGraph[node].keys():
                        try:
                            plt.plot([node.x,connectedNode.x],[node.y,connectedNode.y],nodeSymbol)
                        except AttributeError:
                            plt.plot([node.x],[node.y],nodeSymbol)
        print("Done plotting")
        plt.show()

"""MATH STUFF"""
# Mine class keeps track of mine position and radii      
class Mine:
    numMines = 0
    mines = []
    def __init__(self,centerX,centerY,radius,color:str='',name:str=''):
        Mine.numMines += 1
        self.number=Mine.numMines
        if len(name) > 0:
            self.name = name
        else:
            self.name = f'MID: ' + str(self.numMines)
        
        if len(color) > 0:
            self.color = color
        else:
            self.color = np.random.random(),np.random.random(),np.random.random()
        self.x, self.y, self.radius = np.round(centerX,2) , np.round(centerY,2), np.round(radius,2)
        # Node storage
        self.nodes = [] 
        self.connectedMines = []
        self.overlappingMines = []
        Mine.mines.append(self)
        
    def getPos(self):
        return self.x,self.y
    def getRadius(self):
        return self.radius
    def getNodes(self):
        return self.nodes    

    def addNode(self,node:"Node"):
        #Node.nodes.append(node)
        self.nodes.append(node)
        #self.nodes.append(node) 
        return node
    def connectMineNodes(self):
        sortedNodes = sorted(self.nodes, key=lambda node: node.angle)

        #delete connections
        for node in self.nodes:
            for connection in node.connections:
                if connection.connectionType==seg.ARC:
                    connection.deleteConnection()
        #print(f"length : {len(sortedNodes)}")
        for i in range(len(sortedNodes)-1):
            #print(f"i:{i},i2:{i+1}")
            arcConnection = Connection(sortedNodes[i],sortedNodes[i+1])
            if(arcConnection.validPath()):
                arcConnection.addGraph()
        arcConnection = Connection(sortedNodes[len(sortedNodes)-1],sortedNodes[0])
        if(arcConnection.validPath()):
            arcConnection.addGraph()
       
        #TODO: validate that path doesn't run through another mine. Validation isn't setup yet

        return getrefcount(self)

    def __str__(self):
        return self.name
    def __repr__(self):
        return self.__str__()

# Node class keeps track of node positions
class Node:
    nodeNum = 0
    connectionList = []
    nodeGraph={}
    def __init__(self, xPosition: float, yPosition:float,floating:bool,name:str=""):
        Node.nodeNum += 1
        if len(name) < 1:
            self.name = "NID: "+str(Node.nodeNum)
        else:
            self.name = name 
        
        self.connections= [] #this list makes things much easier when checking all connections
        self.x = xPosition
        self.y = yPosition
        self.plotted = False # To prevent hopefully duplicate plotting
        self.terminated = False
        self.nodeGraph.update({self:None})
        self.parentMine=None
        self.floating=floating

    # Establishes a connection between nodes
    # Does not add it to the nodegraph yet however
    def connectNode(self,node:"Node") -> Connection:
        if(self==node):
            raise TypeError("Same nodes")
        nodeConnection=Connection(self,node) #connection initialization 
        
        self.connections.append(nodeConnection)
        node.connections.append(nodeConnection)
        #self.connected = True
        #node.connected = True
        """
        if node.parentMine not in self.parentMine.connectedMines and self.parentMine not in node.parentMine.connectedMines:
            self.parentMine.connectedMines.append(node.parentMine)
            node.parentMine.connectedMines.append(self.parentMine)

        """
        return nodeConnection

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
    
    def __init__(self,parentMine:"Mine"=None,targetMine:"Mine"=None,internal:bool=True,primary:bool=True,name:str=''):
        Node.nodeNum += 1
        if len(name) < 1:
            self.name = "FNID: "+ str(Node.nodeNum)
            if(Node.nodeNum==168):
                print("WTFFF")
        else:
            self.name = name 
        self.parentMine = parentMine
        self.x = 0.0
        self.y = 0.0
        self.angle=0.0
        self.connected = False
        self.plotted = False # To prevent hopefully duplicate plotting
        self.targetMine = targetMine
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
            
            internalAngle = np.arccos(np.clip(((parentMine.radius)+(targetMine.radius))/d,-1,1))
            """
            if internalCos < 1:
            else:
                self.terminated = True
                return None
            occasionally breaks code
            """
            if primary:
                self.angle=internalAngle+offsetAngle
                self.x = ((parentMine.radius) * np.cos(self.angle)) + parentMine.x
                self.y = ((parentMine.radius) * np.sin(self.angle)) + parentMine.y
            else:
                self.angle=internalAngle-(offsetAngle)
                self.x = (parentMine.radius) * np.cos(self.angle) + parentMine.x
                self.y = (parentMine.radius) * np.sin(self.angle+np.pi) + parentMine.y
            
        else:
            # Create external angle
            externalAngle = np.arccos(np.abs(parentMine.radius-targetMine.radius)/d)

            if primary:
                self.angle=externalAngle+offsetAngle
                self.x = ((parentMine.radius) * np.cos(self.angle)) + parentMine.x
                self.y = ((parentMine.radius) * np.sin(self.angle)) + parentMine.y
            else:
                self.angle=externalAngle-offsetAngle+np.pi
                self.x = (parentMine.radius) * np.cos(self.angle-np.pi) + parentMine.x
                self.y = (parentMine.radius) * np.sin(self.angle) + parentMine.y
        
        self.angle=np.atan2(self.y-parentMine.y,self.x-parentMine.x)


        self.x = round(self.x,3)
        self.y = round(self.y,3)
        super().__init__(self.x,self.y,False,self.name)
        self.parentMine = parentMine #VERY NECESSARY DO NOT REMOVE
        if len(name) < 1:
            self.name = "CID:"+str(parentMine.number)+"."+str(Node.nodeNum)
        else:
            self.name = name 

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

    def getTargetMine(self) -> Mine:
        return self.targetMine
    def __str__(self):
        return self.name
    def __repr__(self):
        return self.__str__()


if __name__=="__main__":
    numMines = 10
    radius = 16
    debug = False
    xMin = -numMines*radius
    xMax = numMines*radius
    yMin = -numMines*radius
    yMax = numMines*radius
    if not debug:
        field = Field(xMin,xMax,yMin,yMax)
    else:
        field = Field(-200,200,-300,300)
    genXMin = -radius*(numMines//2)
    genXMax = radius*(numMines//2)
    genYMin =-radius*(numMines//2)
    genYMax = radius*(numMines//2)
    position = [0,0] 
    mineGenTolerance = 0*radius

    # Mine generation, do not add floating nodes before this point
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
            
            print("added a mine")
        print("done adding mines, connecting nodes on mine")


        #print(Node.nodeGraph)
            
        for mine in field.mines:
            mine.connectMineNodes()
    
        ## Add floating nodes after this point##


        

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
        """
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
        """

        for mine in field.mines:
            print(mine,'connected to',','.join(m.__str__() for m in mine.connectedMines))

            mine.connectMineNodes()
        startNode=field.placeStartNode(0,(genYMin-radius)-20)
        endNodes=field.placeEndNodes(genXMin,genXMax,(genYMax+radius)+20,10)
        solverGraph=genPathFromNodes.Graph(Node.nodeGraph)
        shortestPath=solverGraph.shortest_path(startNode,endNodes)
        print("Shortest Path:")
        print(shortestPath)
        #newgraph=basicDijkstras.Graph(Node.nodeGraph)
        #print("AHHHHHHHHHHH")
        #print(list(Node.nodeGraph.keys())[0])
        #print(list(Node.nodeGraph.keys())[10])
        #print(newgraph.shortest_path(list(Node.nodeGraph.keys())[0],list(Node.nodeGraph.keys())[10]))

    field.plotField(False)
"""
TODO:
X=Done
- = Todo
 X Arc lengths
 X Terminate nodes if the nodes themselves are created within another mine
 X Establish a list of paths between connected Nodes
 - Terminate internal bitangents unless external bitangents are intersecting ???
 X Combine all node lists into one
 - Generate Hugging Nodes
 X Generate Floating Nodes
 - Termination Way that can help optimize generation?:
    + check if a pair of nodes can exist before actually creating them.
"""
import matplotlib.pyplot as pyplot
import matplotlib.collections as mc
import numpy as np
"""
Each mine requires at least 4 nodes that connect to another mine's other 4 nodes.
Ex:
    mine1 = Mine(centerX,centerY,radius)
    mine2 = Mine(centerX,centerY,radius)

    Node Setup:
        Node(parent,target,internal(T/F),primary(T/F))

    internal/external Bitangents:
      - internal:True
      - external:False
    Primary/Secondary Points
      - Primary:True
      - Secondary:False

    Repeat for each mine:
        Rememebr to swap parent and target depending on how 
        you want to connect the mines

        mine1InterNode1 = Node(mine1,mine2,True,True)
        mine1InterNode2 = Node(mine1,mine2,True,False)
        mine1ExterNode1 = Node(mine1,mine2,False,True)
        mine1ExterNode2 = Node(mine1,mine2,False,False)
"""

"""MATH STUFF"""
# Mine class keeps track of mine position and radii
class Mine:
    numMines = 0
    def __init__(self,centerX,centerY,radius):
        self.x, self.y, self.radius = float(centerX) , float(centerY), float(radius)
        Mine.numMines += 1
        self.nodes = []
        
    def getPos(self):
        return self.x,self.y
    def getRadius(self):
        return self.radius
    def getNodes(self):
        return self.nodes
    def addNode(self,node:"Node"):
        self.nodes.append(node)
    def __str__(self):
        return f'pos: {self.x,self.y} radius:{self.radius}'
    def __repr__(self):
        return self.__str__()

# Node class keeps track of node positions
class Node:

    def __init__(self,parentMine:"Mine",targetMine:"Mine",internal:bool=True,primary:bool=True):
        self.parentMine = parentMine
        self.connectedNodes = []
        self.x = 0.0
        self.y = 0.0
        self.__targetMine = targetMine
        parentMine.addNode(self)
        d = np.sqrt((parentMine.x-targetMine.x)**2+(parentMine.y-targetMine.y)**2) # Algabraic Distance Formula

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
            internalAngle = np.arccos(((parentMine.radius)+(targetMine.radius))/d)
            
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

    def distanceFromNode(self,node:"Node"):
        distance = 0
        if len(self.connectedNodes) > 0 and node in self.connectedNodes:
            distance = np.sqrt((self.x-node.x)**2+(self.y-node.y)**2)
        return distance

    def connectNode(self,node:"Node"):
        self.connectedNodes(node)
        node.connectedNodes(self)
        return node

    def get_pos(self):
        return (round(float(self.x),3),round(float(self.y),3))

    def getTargetMine(self):
        return self.__targetMine
    def __str__(self):
        return f"Parent Mine: {self.parentMine}"
    def __repr__(self):
        return self.__str__()


sample = Mine(25,25,16)
otherSample = Mine(0,100,16)
mine = Mine(100,75,16)

"""""""""Internal Bitangents"""""""""
# sample and other sample connections
sampleNode1 = Node(sample,otherSample,True)
sampleNode2 = Node(sample,otherSample,True,False)

otherSampleNode1 = Node(otherSample,sample,True)
otherSampleNode2 = Node(otherSample,sample,True,False)

# mine and sample connections
mineToSampleInterNode1 = Node(mine,sample,True)
mineToSampleInterNode2 = Node(mine,sample,True,False)

sampleToMineInterNode1 = Node(sample,mine,True)
sampleToMineInterNode2 = Node(sample,mine,True,False)

# mine and other sample connections
mineToOtherSampleInterNode1 = Node(mine,otherSample,True)
mineToOtherSampleInterNode2 = Node(mine,otherSample,True,False)

otherSampleToMineInterNode1 = Node(otherSample,mine,True)
otherSampleToMineInterNode2 = Node(otherSample,mine,True,False)

"""""""""External Bitangents"""""""""
# sample and other sample connections
sampleExternalNode1 = Node(sample,otherSample,False)
sampleExternalNode2 = Node(sample,otherSample,False,False)

sampleExternalNode3 = Node(otherSample,sample,False)
sampleExternalNode4 = Node(otherSample,sample,False,False)

# mine and sample connections
mineToSampleExternNode1 = Node(mine,sample,False)
mineToSampleExternNode2 = Node(mine,sample,False,False)

sampleToMineExternNode1 = Node(sample,mine,False)
sampleToMineExternNode2 = Node(sample,mine,False,False)

# mine and other sample connections
mineToOtherSampleExternNode1 = Node(mine,otherSample,False)
mineToOtherSampleExternNode2 = Node(mine,otherSample,False,False)

otherSampleToMineExternNode1 = Node(otherSample,mine,False)
otherSampleToMineExternNode2 = Node(otherSample,mine,False,False)

# # Connect the Nodes
# sampleNode1.connectNode(otherSampleNode1)
# sampleNode2.connectNode(otherSampleNode2)

# sampleExternalNode1.connectNode(sampleExternalNode3)
# sampleExternalNode2.connectNode(sampleExternalNode4)
""" MATPLOTLIB STUFF"""
plt = pyplot

# Circles/Mines
sampleCircle = pyplot.Circle(sample.getPos(),sample.getRadius(),color='r')
otherSampleCircle = pyplot.Circle(otherSample.getPos(),otherSample.getRadius(),color='b')
mineCircle = pyplot.Circle(mine.getPos(),mine.getRadius(),color="black")

fig, ax = pyplot.subplots()
ax.set_aspect("equal")
ax.add_patch(sampleCircle)
ax.add_patch(otherSampleCircle)
ax.add_patch(mineCircle)

"""""""""Plot Internal Bitangents"""""""""
# sample and other sample internal connect
plt.plot([sampleNode1.x,otherSampleNode1.x],[sampleNode1.y,otherSampleNode1.y],color="y")
plt.plot([otherSampleNode2.x,sampleNode2.x],[otherSampleNode2.y,sampleNode2.y],color='g')

# mine and sample internal connect
plt.plot([mineToSampleInterNode1.x,sampleToMineInterNode1.x],[mineToSampleInterNode1.y,sampleToMineInterNode1.y],color='y')
plt.plot([mineToSampleInterNode2.x,sampleToMineInterNode2.x],[mineToSampleInterNode2.y,sampleToMineInterNode2.y],color='g')

# mine and other sample internal connect
plt.plot([mineToOtherSampleInterNode1.x,otherSampleToMineInterNode1.x],[mineToOtherSampleInterNode1.y,otherSampleToMineInterNode1.y],color='y')
plt.plot([mineToOtherSampleInterNode2.x,otherSampleToMineInterNode2.x],[mineToOtherSampleInterNode2.y,otherSampleToMineInterNode2.y],color='g')

"""""""""Plot External Bitangents"""""""""
# sample and other sample external connect
plt.plot([sampleExternalNode1.x,sampleExternalNode4.x],[sampleExternalNode1.y,sampleExternalNode4.y],color='c')
plt.plot([sampleExternalNode2.x,sampleExternalNode3.x],[sampleExternalNode2.y,sampleExternalNode3.y],color='m')

""" 
#For whatever reason, a bug where one of the external nodes is displayed slightly off.
#But, the node position itself through printing to console appears to be correctly calculated.
"""
# mine and sample external connect
plt.plot([mineToSampleExternNode1.x,sampleToMineExternNode2.x],[mineToSampleExternNode1.y,sampleToMineExternNode2.y],color='c')
plt.plot([mineToSampleExternNode2.x,sampleToMineExternNode1.x],[mineToSampleExternNode2.y,sampleToMineInterNode1.y],color='m')

# mine and other sample external connect
plt.plot([mineToOtherSampleExternNode1.x,otherSampleToMineExternNode2.x],[mineToOtherSampleExternNode1.y,otherSampleToMineExternNode2.y],color='c')
plt.plot([mineToOtherSampleExternNode2.x,otherSampleToMineExternNode1.x],[mineToOtherSampleExternNode2.y,otherSampleToMineExternNode1.y],color='m')

plt.xlim(-100,150)
plt.ylim(-100,150)
plt.show()

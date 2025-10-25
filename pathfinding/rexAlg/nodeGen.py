import matplotlib.pyplot as pyplot
import matplotlib.collections as mc
import numpy as np

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

class Node:

    def __init__(self,parentMine:"Mine",targetMine:"Mine",internal:bool=True,primary:bool=True):
        self.parentMine = parentMine
        self.connectedNodes = []
        self.x = 0.0
        self.y = 0.0
        self.__targetMine = targetMine
        d = np.sqrt((parentMine.x-targetMine.x)**2+(parentMine.y-targetMine.y)**2)
        self.primary= primary # Primary node is the first node where it is placed towards the top of the circle

        # Create Angle Offset(relative to target mine)
        if parentMine.y > targetMine.y:
               offsetAngle =  np.arccos(np.clip((parentMine.x-targetMine.x)/d,-1,1))+np.pi
        elif parentMine.y < targetMine.y:
            offsetAngle = -np.arccos(np.clip((parentMine.x-targetMine.x)/d,-1,1))+np.pi
        elif parentMine.y == targetMine.y:
            if parentMine.x < targetMine.x:
                offsetAngle =  np.arccos(np.clip((parentMine.x-targetMine.x)/d,-1,1))+np.pi
            elif parentMine.x > targetMine.x:
                offsetAngle = -np.arccos(np.clip((parentMine.x-targetMine.x)/d,-1,1))+np.pi

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

    def getTargetMine(self):
        return self.__targetMine
    def __str__(self):
        return f"Parent Mine: {self.parentMine}"
    def __repr__(self):
        return self.__str__()
    

sample = Mine(25,25,16)
otherSample = Mine(0,100,16)

# Internal Bitangents
sampleNode1 = Node(sample,otherSample,True)
sampleNode2 = Node(sample,otherSample,True,False)

otherSampleNode1 = Node(otherSample,sample,True)
otherSampleNode2 = Node(otherSample,sample,True,False)

# External Bitangents
sampleExternalNode1 = Node(sample,otherSample,False)
sampleExternalNode2 = Node(sample,otherSample,False,False)

sampleExternalNode3 = Node(otherSample,sample,False)
sampleExternalNode4 = Node(otherSample,sample,False,False)

plt = pyplot

# Circles/Mines
sampleCircle = pyplot.Circle(sample.getPos(),sample.getRadius(),color='r')
otherSampleCircle = pyplot.Circle(otherSample.getPos(),otherSample.getRadius(),color='b')

fig, ax = pyplot.subplots()
ax.set_aspect("equal")
ax.add_patch(sampleCircle)
ax.add_patch(otherSampleCircle)

# Plot Internal Bitangents
plt.plot([sampleNode1.x,otherSampleNode1.x],[sampleNode1.y,otherSampleNode1.y],color="y")
plt.plot([otherSampleNode2.x,sampleNode2.x],[otherSampleNode2.y,sampleNode2.y],color='g')

# Plot External Bitangents
plt.plot([sampleExternalNode1.x,sampleExternalNode4.x],[sampleExternalNode1.y,sampleExternalNode4.y],color='c')
plt.plot([sampleExternalNode2.x,sampleExternalNode3.x],[sampleExternalNode2.y,sampleExternalNode3.y],color='m')

plt.xlim(-100,150)
plt.ylim(-100,150)
plt.show()

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

class Node():

    def __init__(self,parentMine:"Mine",targetMine:"Mine",internal:bool=True):
        self.parentMine = parentMine
        self.connectedNodes = []
        self.x = 0.0
        self.y = 0.0
        parentMine.addNode(self)

        if internal:
            self.__connectToMineInternal(targetMine)
        else:
            self.__connectToMineExternal(targetMine)
        
  
    def __connectToMineInternal(self,mine:"Mine"):
        self.connectedNodes.append(mine)
        pMinePos = self.parentMine.getPos()
        cMinePos = mine.getPos()
        distanceBetweenMines = np.sqrt((pMinePos[0]-cMinePos[0])**2+(pMinePos[1]-cMinePos[1])**2)
        if cMinePos[1] < pMinePos[1]:
            theta = 2*(np.pi) - np.arccos((np.sqrt(self.parentMine.getRadius())+np.sqrt(mine.getRadius()))/distanceBetweenMines)
        else:
            theta = np.arccos((np.sqrt(self.parentMine.getRadius())+np.sqrt(mine.getRadius()))/distanceBetweenMines)

        self.x = self.parentMine.getRadius() * np.cos(theta) + pMinePos[0]
        self.y = self.parentMine.getRadius() * np.sin(theta) + pMinePos[1]

    def __connectToMineExternal(self,mine:"Mine"):
        self.connectedNodes.append(mine)
        pMinePos = self.parentMine.getPos()
        cMinePos = mine.getPos()
        distanceBetweenMines = np.sqrt((pMinePos[0]-cMinePos[0])**2+(pMinePos[1]-cMinePos[1])**2)
        if cMinePos[1] < pMinePos[1]:
            theta = 2*(np.pi) - np.arccos(abs(self.parentMine.getRadius()-mine.getRadius())/distanceBetweenMines)
        else:
            np.arccos(abs(self.parentMine.getRadius()-mine.getRadius())/distanceBetweenMines)
        print(f'{theta:.2f}')
        self.x = self.parentMine.getRadius() * np.cos(theta) + pMinePos[0]
        self.y = self.parentMine.getRadius() * np.sin(theta) + pMinePos[1]
        
    def distanceFromNode(self,node:"Node")->int:
        distance = 0
        if len(self.connectedNodes) > 0 and node in self.connectedNodes:
            distance = np.sqrt((self.x-node.x)**2+(self.y-node.y)**2)
        return distance

    def __str__(self):
        return f"Parent Mine: {self.parentMine}"
    def __repr__(self):
        return self.__str__()
    

sample = Mine(25,25,16)
otherSample = Mine(100,100,16)
print(sample.x,sample.y)
print(otherSample.x,otherSample.y)
sampleNode = Node(sample,otherSample)
otherSampleNode = Node(otherSample,sample)

plt = pyplot

circle1 = pyplot.Circle(sample.getPos(),sample.getRadius(),color='r')
circle2 = pyplot.Circle(otherSample.getPos(),otherSample.getRadius(),color='b')

fig, ax = pyplot.subplots()

ax.add_patch(circle1)
ax.add_patch(circle2)

plt.plot([sampleNode.x],[sampleNode.y],'o',color="b")

plt.xlim(-100,150)
plt.ylim(-100,150)
plt.show()

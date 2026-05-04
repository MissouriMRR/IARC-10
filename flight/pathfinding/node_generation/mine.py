from node_generation import MineNode, Node,Connection
import random
import numpy as np
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
        self.mineDistances={} #Distance of this mine to all other mines, used for checking for overlapping path at arbitrary radius.
        
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
            sortedNodes[i].mineOrder = i
            arcConnection = Connection(sortedNodes[i],sortedNodes[i+1])
            if(arcConnection.validPath()):
                arcConnection.addGraph()
        arcConnection = Connection(sortedNodes[len(sortedNodes)-1],sortedNodes[0])
        if(arcConnection.validPath()):
            arcConnection.addGraph()
            


    def __str__(self):
        return self.name
    def __repr__(self):
        return self.__str__()
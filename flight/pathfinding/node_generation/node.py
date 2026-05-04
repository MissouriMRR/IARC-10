from node_generation import Field, Mine, Node, MineNode, Connection
# Node class keeps track of node positions
class Node:
    nodeNum = 0
    connectionList = []
    
    def __init__(self, xPosition: float, yPosition:float,floating:bool,angle:float=0,name:str="",labeled:bool=False, nType:str="default"):
        """
        Create node based off of (x,y) coordinate, whether or not it is floating,
        its angle, optional name, whether or not it is named, labeled, 
        and the kind of Node it is(for selective elimination purposes)
        \nTypes(case sensitive):\n "default","start","end"
        """
        Node.nodeNum += 1
        if len(name) < 1:
            self.name = "FNID: "+str(Node.nodeNum)
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

        # For selective elimination(dont want to delete end or start nodes)
        # Types:
        # - "default"
        # - "start"
        # - "end"
        self.nType = nType

   
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
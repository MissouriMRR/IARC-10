from node_generation import Mine, Node, MineNode, Connection,seg
import quads
from typing import Callable
import numpy as np
from matplotlib import pyplot
import random
# Field generates nodes off of mines, generates mines too
class Field:
    mines = []
    
    debugPoints = [] # purely for debuging and testing, field will plot these points

    # simFieldSize = simulated size of field, a rectangle.
    # fieldCorners = arbitrary corners that might not form a rectangle
    def __init__(self,simFieldSize:list,fieldCorners:list):
        """
        simFieldSize = simulated size of field, a rectangle's [width,height].
        \nfieldCorners = arbitrary corners of field, a quadrilateral of four corners
        """
        self.nodeGraph={}
        
        simCorners = [(0,simFieldSize[1]),
              (simFieldSize[0],simFieldSize[1]),
              (0,0),
              (simFieldSize[0],0)]
        self.rawCorners = fieldCorners

        # For simulation bounded view
        self.simVertPairLeft = [simCorners[0],simCorners[2]]
        self.simVertPairRight = [simCorners[1],simCorners[3]]
        self.simHorzPairUpper = [simCorners[2],simCorners[3]]
        self.simHorzPairLower = [simCorners[0],simCorners[1]]
        # For field bounds
        self.fieldVertPairLeft = [self.rawCorners[0],self.rawCorners[2]]
        self.fieldVertPairRight = [self.rawCorners[1],self.rawCorners[3]]
        self.fieldHorzPairUpper = [self.rawCorners[0],self.rawCorners[1]]
        self.fieldHorzPairLower = [self.rawCorners[2],self.rawCorners[3]]
        
        self.xMin = min(simCorners[0][0],simCorners[2][0])
        self.xMax = max(simCorners[1][0],simCorners[3][0])
        self.yMin = min(simCorners[2][1],simCorners[3][1])
        self.yMax = min(simCorners[0][1],simCorners[1][1])

        # To be used for comparing if nodes are within the valid field
        self.leftLine, self.leftSlope = Field.getLine(self.fieldVertPairLeft[0],self.fieldVertPairLeft[1])
        self.rightLine, self.rightSlope = Field.getLine(self.fieldVertPairRight[0],self.fieldVertPairRight[1])
        self.upperLine, self.upperSlope = Field.getLine(self.fieldHorzPairUpper[0],self.fieldHorzPairUpper[1])
        self.lowerLine, self.lowerSlope = Field.getLine(self.fieldHorzPairLower[0],self.fieldHorzPairLower[1])
        
        self.floatingNodes= [] #List of floating nodes, necessary so we can connect floating nodes to mines, if mines are created afterwards.
        self.mines = []
        self.mineQuadTree= quads.QuadTree((self.xMin+self.xMax/2,self.yMin+self.yMax/2),self.xMax,self.yMax) # Used for collision detection, holds mines
        Connection.field=self


    # This type of node will not have a parent mine, primarily used for start/end points
    
    def addFloatingNode(self,x:float,y:float,ndType:str=None) ->'Node':
        """
        Given a coordinate position, place a floating node onto the field
        """
        fNode = Node(x,y,True,nType=ndType) # Floating Node
        
        for mine in Connection.field.mines:
            mineNodePrimary = MineNode(parentMine=mine,floatingNode=fNode,primary=True,connectedToFloating=True)
            mineNodeSecondary = MineNode(parentMine=mine, floatingNode=fNode, primary=False,connectedToFloating=True)
            mine.addNode(mineNodePrimary)
            mine.addNode(mineNodeSecondary)
            mineNodePrimary.connectNode(fNode)
            mineNodeSecondary.connectNode(fNode)
        
        # if fNode in self.nodeGraph:
        #     if len(self.nodeGraph[fNode])==0:
        #         del self.nodeGraph[fNode]
        return fNode
    
    #Due to the current node stucture, right now this only modifies the nodeGraph
    def placeStartNode(self,xVal:float ,yVal:float ) -> 'Node':
        """
        Given a coordinate, place a start node onto the field
        """
        start = self.addFloatingNode(xVal,yVal,"start")
        self.floatingNodes.append(start)
        return start    
    # Places density amount of end nodes equidistance along the y coordinate and between xMin and xMax
    def placeEndNodesLine(self, yVal: float, density: int):
        """
        Given a y-value and density amount of nodes, places the end Nodes onto the field
        """
        returnList=[]
        if density > 1:
            xVals = [self.xMin + (i * ((self.xMax-self.xMin)/density-1)) for i in range(density)]
            for x in xVals:
                returnList.append(self.addFloatingNode(x,yVal,"end"))
        else:
            returnList.append(self.addFloatingNode((self.xMin+self.xMax)/2,yVal,"end"))
        self.floatingNodes+=returnList
        return returnList
    
    def placeEndNodesPositions(self,position: list[tuple[float,float]]):
        """
        Given a list positions [(x,y)..]
        \nPlace end Nodes at those points
        """
        returnList = []
        for pos in position:
            returnList.append(self.addFloatingNode(pos[0],pos[1],"end"))
        return returnList
    def addMine(self,centerX:float,centerY:float,radius:int,color:str=''):
        """
        Given the simulated local coordinates, radius, and optional color;
        add a new Mine object centered at the coordinates to the field and generate/regenerate nodes and connections
        """
        newMine = Mine(centerX,centerY,radius,color=color)
        self.mines.append(newMine)
        self.mineQuadTree.insert((centerX,centerY), data=newMine)
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
            mine.mineDistances[target] = np.sqrt((mine.x-target.x)**2 + (mine.y-target.y)**2)
            target.mineDistances[mine] = mine.mineDistances[target]


        for fNode in self.floatingNodes:
            mineNodePrimary = MineNode(parentMine=newMine,floatingNode=fNode,primary=True,connectedToFloating=True)
            mineNodeSecondary = MineNode(parentMine=newMine, floatingNode=fNode, primary=False,connectedToFloating=True)
            newMine.addNode(mineNodePrimary)
            newMine.addNode(mineNodeSecondary)
            mineNodePrimary.connectNode(fNode)
            mineNodeSecondary.connectNode(fNode)
        shallowCopy=self.nodeGraph.copy()
        # Check if any of the other node connections pass through the newly created mine.
        for node1 in [n for n in shallowCopy.keys() if n not in newMine.nodes]:
           deepCopy=shallowCopy[node1].copy()
           for node2 in deepCopy:
                connection=Connection(node1,node2)
                if(connection.mineCollision(newMine)):
                    connection.deleteConnection()
         
    @staticmethod #Given two points, get the line equation and slope (to determine negative or positive slope)
    def getLine(point1: tuple, point2: tuple) -> tuple[Callable[[float], float], float]:
        """
        Given two points as a tuple of floats each, get a line function and its slope
        """
        x1 = point1[0]
        y1 = point1[1]
        x2 = point2[0]
        y2 = point2[1]
        try:
            slope = (y2-y1)/(x2-x1)
        except ZeroDivisionError: # Infinite/Vertical slope
            # x means nothing in this case, for all values of Y, its x is x1 and x2
            return (lambda x: x1 + 0*x, "undef") 

        offset = y2-slope*x2

        f = lambda x: (slope*x)+offset
        return (f,slope)
    
    # Given a line function and a point, detect if the point   
    @staticmethod # lies to the left of the line
    def isPointLeftofLine(line:Callable[[tuple[float,float],tuple[float,float]],tuple[Callable[[float],float],float]], slope:float, point: tuple[float,float]) -> bool:
        """
        Given a line function, point, and slope;
        Check if the point lies to the left of the line
        """
        """
        If the slope between p1 and p2 is negative, p3's y-value must be 
        below the line for it to be to the left of line
        p1
         `
          `
           `
            `
          p3 `
              `
              p2
        The logic will be adjusted for positive and undefined(vertical line) slope.
        
        """
        x = point[0]
        y = point[1]
        if isinstance(slope,str):
            if slope == "undef": # Verticle line
                if (x < line(x)):
                    return True
        if isinstance(slope,float):
            if slope < 0: # Negative slope
                if (y < line(x)):
                    return True
            elif slope > 0: # Positive slope
                if (y > line(x)):
                    return True
            else:
            # If the points are horizontal, and since this is checking a *line*
            # A point will always be within the line <-----*--->
            # So technically cant be left of the line
                return False
        return False
    # Given a line function and a point, detects if the point
    @staticmethod # lies to the right of the line
    def isPointRightofLine(line:Callable[[tuple[float,float],tuple[float,float]],tuple[Callable[[float],float],float]], slope:float, point: tuple[float,float]) -> bool:
        """
        Given a line function, point, and slope;
        Check if the point lies to the right of the line
        """
        """
        If the slope between p1 and p2 is negative, p3's y-value must be 
        above the line for it to be to the right of line
        p1
         `
          ` p3
           ` 
            `
             `
              `
              p2
        The logic will be adjusted for positive and undefined(vertical line) slope
        """
        # point[0],point[1] = x,y
        x = point[0]
        y = point[1]
        if isinstance(slope,str):
            if slope == "undef": # Verticle line
                if (x > line(x)):
                    return True
        if isinstance(slope,float):
            if slope < 0: # Negative Slope
                if (y > line(x)):
                    return True
            elif slope > 0: # Positive Slope
                if (y < line(x)):
                    return True
            else: # Horizontal
                return False
        return False
    
    # Given a line function and a point, detects if the point
    @staticmethod # lies above the line
    def isPointAboveLine(line:Callable[[tuple[float,float],tuple[float,float]],tuple[Callable[[float],float],float]],slope:float, point:tuple[float,float]):
        """
        Given a line function, point, and slope;
        Check if the point lies above the line
        """
        x = point[0]
        y = point[1]
        if isinstance(slope,str):
            if slope == "undef": # Vertical line
                return True
        if isinstance(slope,float):
            if (y > line(x)):
                return True
        return False
    # Given a line function and a point, detects if the point
    @staticmethod # lies below the line
    def isPointBelowLine(line:Callable[[tuple[float,float],tuple[float,float]],tuple[Callable[[float],float],float]],slope:float, point:tuple[float,float]):
        """
        Given a line function, point, and slope;
        Check if the point lies below the line
        """
        x = point[0]
        y = point[1]
        if isinstance(slope,str):
            if slope == "undef": # Vertical line
                return None
        if isinstance(slope,float):
            if (y < line(x)):
                return True
        return False
    
    # Purely for debugging will have a growing list of parameters
    def plotField(self,labeled:bool=False,path:list["Node"]=[],title:str="",xlabel:str="",labelPath:bool=False, pastPath:list["Node"] = []) -> None:
        """
        Using the matplotlib library and various optional debug options, plots the current iteration of the field
        """
        plt = pyplot
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        padding = 10
        plt.xlim(self.xMin-padding,self.xMax+padding)
        plt.ylim(self.yMin-padding,self.yMax+padding)

        if len(title) <= 0:
            title = f"Mines({len(self.mines)}) and Potential Paths"
        xlabel += "KEY:\n"
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
        if len(path) > 0:
            xlabel += "\nBlack = A* path"
            if labelPath:
                for node in path:
                    node.labeled = True
        
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
        xlabel += "Colors = Potential Paths"
        xlabel += "\nLight Gray = Simulated Boundary"
        xlabel += "\nDark Gray = Field Boundary"
        xlabel += "\nX = Mines' centers"
        # If a path is passed in, display the path as a black line
        if len(path) > 0:
            for i, node in enumerate(path):
                if (i < len(path)-1):
                    nextNode = path[i+1]
                    plt.plot([node.x,nextNode.x],[node.y,nextNode.y],color=(0,0,0))
                        
        if len(Field.debugPoints) > 0: # Points that are plotted for debugging only
            print("Plotting debug points")
            for point in Field.debugPoints:
                plt.plot(point[0],point[1],"o",color=(0,0,0))

        # Plot simulation boundaries
        for pair in [self.simHorzPairUpper,self.simHorzPairLower,self.simVertPairLeft,self.simVertPairRight]:
            plt.plot([pair[0][0],pair[1][0]],[pair[0][1],pair[1][1]],color = (0.5,0.5,0.5))
        # Plot field boundaries
        for pair in [self.fieldVertPairLeft,self.fieldVertPairRight,self.fieldHorzPairUpper,self.fieldHorzPairLower]:
            plt.plot([pair[0][0],pair[1][0]],[pair[0][1],pair[1][1]],color = (0.3,0.3,0.3))
        
        # Plot the previous, if any, path from a previous iteration
        # Useful if you run plotField twice in the same program instance.
        if len(pastPath) > 0:
            for i, node in enumerate(pastPath):
                if (i < len(path)-1):
                    nextNode = path[i+1]
                    plt.plot([node.x,nextNode.x],[node.y,nextNode.y],color=(0,0,0))

        print("Done plotting")
        print("Displaying field...")
        
        plt.title(title)
        plt.xlabel(xlabel)
        plt.show()
        print("Done displaying field.")

    # Run this to remove nodes that have no associated connection, ie, {node: None}
    def cleanNodeGraph(self):
        """
        Removes nodes that have no associated connection from the node graph
        \nSuch as:
        \n{node: None}
        """
        if self.nodeGraph != None:
            for node in self.nodeGraph.copy():
                if self.nodeGraph[node] == None:
                    del self.nodeGraph[node]
            return self.nodeGraph
        else:
            print("Node graph is empty")


    def graphAtRadius(self,radius:int):
        shallowCopy=self.nodeGraph.copy()
        for node1 in shallowCopy.keys():
            deepCopy=shallowCopy[node1].copy()
            shallowCopy[node1]=deepCopy

        for node1 in shallowCopy.keys():
           deepCopy=shallowCopy[node1].copy()
           for node2 in deepCopy:
                connection=Connection(node1,node2)
                if(connection.connectionType==seg.ARC):
                    connection.deleteConnection()
        
    def increaseRadius(self,step:int):
        """
        Manually increases radius of all mines by a step 
        and recalculates connections accordingly 
        """
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
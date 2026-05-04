#Simple class which is used in the nodegraph and holds informatation about distance path and type (straight/arc-ed)
#Moved some useful connection functions here as well.
from node_generation import Field, Mine, Node, MineNode
from enum import Enum
import numpy as np
import quads
class seg(Enum):
    ARC=1
    LINE=2

class Connection:
    field:'Field'=None #must be initialized on startup
    def __init__(self,node1: 'Node',node2: 'Node',mineRadius: float=-1):
        self.node1=node1
        self.node2=node2
        self.mineRadius=mineRadius if mineRadius != -1 else Mine.radius # Default to the first mine's radius if not specified, but should be updated to a more dynamic value
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
            
            # Get two different angle differences, one for major arc, the other for minor arc

            nodeAngle1= self.node1.angle
            nodeAngle2= self.node2.angle
            angleTheta=abs(nodeAngle1-nodeAngle2)
            if abs(self.node1.mineOrder-self.node2.mineOrder)==1:
                angleTheta=min(angleTheta,2*np.pi-angleTheta)


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

    def deleteConnection(self,field=None):
        purgeNodes=False

        if field == None:
            field = self.field
            purgeNodes=True

        if self.node1 in field.nodeGraph: 
            if self.node2 in field.nodeGraph[self.node1]:
                del field.nodeGraph[self.node1][self.node2]
            if len(field.nodeGraph[self.node1])==0 and purgeNodes:
                self.node1.deleteNode()
        else:
            self.node1.deleteNode()

        if self.node2 in field.nodeGraph: 
            if self.node1 in field.nodeGraph[self.node2]:
                del field.nodeGraph[self.node2][self.node1]
            if len(field.nodeGraph[self.node2])==0 and purgeNodes:
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
        field = self.field




        """
        Check if the the current connection's nodes are within
        field boundries.
        """
        #  Node landing outside of field boundaries
        """
                               p2
                            `     `   n2
                        `           `
                   Up                  Ri                                       
               `           n1            `
           `                                `
         p1                                   `
            `                                  p4
               `                             `
                  `                        `
                     Le                  Lo
                        `              `
                          `         `
                             `   `   
                               p3(Origin)
        """
        # Node 1 boundary check
        if (self.node1.nType == "default"):
            # Left line check
            if not(field.isPointRightofLine(field.leftLine,field.leftSlope,(x1,y1))):
                # print(x1,y1)
                self.node1.labeled = True
                return False
            # Right line check
            if not(field.isPointLeftofLine(field.rightLine,field.rightSlope,(x1,y1))):
                # print(x1,y1)
                # self.node1.labeled = True
                return False
            # Upper line check
            if not(field.isPointBelowLine(field.upperLine,field.upperSlope,(x1,y1))):
                # print(x1,y1)
                # self.node1.labeled = True
                return False
            # Lower line check
            if not(field.isPointAboveLine(field.lowerLine,field.lowerSlope,(x1,y1))):
                # print(x1,y1)
                # self.node1.labeled = True
                return False
        # Node 2 boundary check
        if (self.node2.nType == "default"):
            # Left line check
            if not(field.isPointRightofLine(field.leftLine,field.leftSlope,(x2,y2))):
                # print(x2,y2)
                self.node1.labeled = True
                return False
            # Right line check
            if not(field.isPointLeftofLine(field.rightLine,field.rightSlope,(x2,y2))):
                # print(x2,y2)
                # self.node1.labeled = True
                return False
            # Upper line check
            if not(field.isPointBelowLine(field.upperLine,field.upperSlope,(x2,y2))):
                # print(x2,y2)
                # self.node1.labeled = True
                return False
            # Lower line check
            if not(field.isPointAboveLine(field.lowerLine,field.lowerSlope,(x2,y2))):
                # print(x2,y2)
                # self.node1.labeled = True
                return False
        
        # Connection intersecting mine test
        
        if self.connectionType == seg.LINE:
            boundingBox=quads.BoundingBox(min_x=min(x1,x2)-self.mineRadius,min_y=min(y1,y2)-self.mineRadius,max_x=max(x1,x2)+self.mineRadius,max_y=max(y1,y2)+self.mineRadius)
            minesToCheck=Connection.field.mineQuadTree.within_bb(boundingBox)
            for mine in minesToCheck:
                
                mine=mine.data
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
            
            for mine in parentMine.mineDistances.keys():
                if mine.mineDistances[parentMine] >= parentMine.radius + mine.radius:
                    continue
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
        if (distance != 0):
            radicalLineDistance: float = ((mine1.radius**2 - mine2.radius**2 + distance**2))/(2*distance)
        else:
            radicalLineDistance: float = 0
       
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
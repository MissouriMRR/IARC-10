import random

from matplotlib.patches import Polygon,Circle
from matplotlib.lines import Line2D
import numpy as np
from flight.pathfinding.nodeField import field
from flight.pathfinding.nodeField.polygonObstacle import PolygonObstacle
from flight.newPathfinding.diamondMine import BlockyObstacle
from flight.pathfinding.nodeField.field import Field, Mine
from flight.pathfinding.nodeField.node_connection import Connection
simSquareWidth, simSquareHeight = 30, 30
blockMatrix = np.zeros((simSquareWidth, simSquareHeight), dtype=int).tolist()
import matplotlib.pyplot as plt

def circle_rect_intersects(
    cx: float,
    cy: float,
    r: float,
    rx: float,
    ry: float,
    rw: float,
    rh: float,
):
    """
    Determine whether a circle intersects a rectangle.

    Parameters:
        cx: x-coordinate of the circle center
        cy: y-coordinate of the circle center
        r: radius of the circle
        rx: x-coordinate of the rectangle's top-left corner
        ry: y-coordinate of the rectangle's top-left corner
        rw: width of the rectangle
        rh: height of the rectangle
    """


    # Closest x on square
    closestX = rx
    if cx >= rx + rw:
        closestX = rx + rw
    elif cx < rx:
        closestX = rx

    # Closest y on square
    closestY = ry
    if cy >= ry + rh:
        closestY = ry + rh
    elif cy < ry:
        closestY = ry

    # Distance from circle center to closest point
    distX = cx - closestX
    distY = cy - closestY
    distance = (distX**2 + distY**2) ** 0.5

    return distance <= r

if __name__ == "__main__":
    points=[]
    curField=Field([simSquareWidth,simSquareHeight],[[0,0],[simSquareWidth,0],[simSquareWidth,simSquareHeight],[0,simSquareHeight]])
    Connection.field=curField
    Mine.radius=2.0
    #random polygon generation
    
    x=random.randint(0,simSquareWidth)
    y=random.randint(0,simSquareHeight)

    for i in range(10):
        points.append((random.randint(0, 200)/100+x, random.randint(0, 200)/100+y))
    polygon1=PolygonObstacle(points,simSquareWidth,simSquareHeight,0,0)
    points=[]
    x=random.randint(0,simSquareWidth)
    y=random.randint(0,simSquareHeight)
    for i in range(10):
        points.append((random.randint(0, 200)/100+x, random.randint(0, 200)/100+y))
    polygon2=PolygonObstacle(points,simSquareWidth,simSquareHeight,0,0)



    
    #plt.imshow(blockMatrix, cmap='gray',origin='lower', extent=[0, simSquareWidth, 0, simSquareHeight])
    lineX=[vertex[0] for vertex in polygon1.vertices]
    lineY=[vertex[1] for vertex in polygon1.vertices]
    """
    for i in polygon1.vertices:
        ax.plot(i[0], i[1], marker='o', markersize=6, linestyle='-', color='green')
    for i in polygon2.vertices:
        ax.plot(i[0], i[1], marker='o', markersize=6, linestyle='-', color='red')
    """
    curField.polygonObstacles.append(polygon1)
    curField.polygonObstacles.append(polygon2)
    polygon1.connectPolygonObstacle(polygon2)
    fig, ax = plt.subplots()
    for i in curField.nodeGraph:
        ax.add_patch(Circle((i.x, i.y), 0.1, color='green'))
        for j in curField.nodeGraph[i]:
            print("Adding line")
            ax.add_line(Line2D([i.x, j.x], [i.y, j.y], color='orange', linewidth=2))

    print(curField.nodeGraph)
    
    ax.plot()

    print("Showing final graph?")
    plt.show()
    """
    centerX, centerY = random.randint(0, simSquareWidth), random.randint(0, simSquareHeight)

    for y in range(len(blockMatrix)):
        for x in range(len(blockMatrix[y])):
            blockMatrix[y][x] = int(circle_rect_intersects(centerX, centerY, 2.0, x, y, 1.0, 1.0))
        
    block=BlockyObstacle(blockMatrix, (0, 0), simSquareWidth, simSquareHeight, 0, 0)
    fig, ax = plt.subplots()
    for i in reversed(blockMatrix):
        print(i)

    plt.imshow(blockMatrix, cmap='gray',origin='lower', extent=[0, simSquareWidth, 0, simSquareHeight])
    poly = Line2D([vertex[0] for vertex in block.vertices], [vertex[1] for vertex in block.vertices], color='orangered', linewidth=3)
    for i in block.vertices:
        ax.plot(i[0], i[1], marker='o', markersize=12, linestyle='-', color='blue')
    # Add to axes
    ax.add_patch(Circle((centerX, centerY), 2.0, color='green', alpha=0.5))
    ax.add_line(poly)
    plt.show()
    """

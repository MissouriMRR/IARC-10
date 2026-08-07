from flight.pathfinding.protoMine import protoMine
import random
if __name__ == "__main__":
    radius=random.randint(1,3)
    x=-random.randint(-100,100)/100
    y=random.randint(-100,100)/100

    print(f"Radius: {radius}")
    print(f"Center Offset: {(x,y)}")
    mine=protoMine(radius,(0.0,0.0),(x,y))
    mine.visualize()
from flight.pathfinding.nodeField.field import Field
import random
from shapely import MultiPoint, Polygon, convex_hull
def polygonObstacleMaker():
    nodeList=[]
    x,y=random.uniform(10,90),random.uniform(10,90)
    for i in range(5):
        nodeList.append((random.uniform(-2,2)+x,random.uniform(-2,2)+y))

    nodeList=convex_hull(MultiPoint(nodeList)).exterior.coords
    return nodeList

if __name__=="__main__":
    arbCorners = [[0,100],[100,100],[0,0],[100,0]]
    # sim_field_size = [width, height]
    sim_field_size = [
        max([arbCorners[0][0], arbCorners[1][0], arbCorners[2][0], arbCorners[3][0]])
        - min([arbCorners[0][0], arbCorners[1][0], arbCorners[2][0], arbCorners[3][0]]),
        max([arbCorners[0][1], arbCorners[1][1], arbCorners[2][1], arbCorners[3][1]])
        - min([arbCorners[0][1], arbCorners[1][1], arbCorners[2][1], arbCorners[3][1]]),
    ]
    simCorners = [
        (0, sim_field_size[1]),
        (sim_field_size[0], sim_field_size[1]),
        (0, 0),
        (sim_field_size[0], 0),
    ]
    fieldSimCoords = {
        "xMin": simCorners[0][0],
        "xMax": simCorners[1][0],
        "yMin": simCorners[3][1],
        "yMax": simCorners[1][1],
    }
    genXMin = int(fieldSimCoords["xMin"])
    genXMax = int(fieldSimCoords["xMax"])
    genYMin = int(fieldSimCoords["yMin"])
    genYMax = int(fieldSimCoords["yMax"])

    field = Field(sim_field_size, arbCorners)
    for i in range(100):
        vertices=polygonObstacleMaker()
        #print([i for i in vertices])
        field.createPolygonObstacle(vertices)
        print(i)
        #field.createPolygonObstacle([(6.542979815844672, 15.561571335630497), (4.902817962129671, 26.215076113067425), (14.927554663001251, 23.94838305195267), (19.643099028315277, 15.827889899755862), (6.542979815844672, 15.561571335630497)])
        #field.createPolygonObstacle([(56.10167276726061, 12.940266507249733), (49.35987046070179, 15.317535026733378), (39.950241154024695, 24.96679760304149), (53.60084858603152, 18.944300144237275), (56.10167276726061, 12.940266507249733)])
        #field.createPolygonObstacle([(35.013800453438805, 10.674310021309719), (29.206855090840104, 18.78026702323732), (29.938451362301105, 18.736911559793704), (40.18822942201414, 14.672568612860271), (35.013800453438805, 10.674310021309719)])
        #field.createPolygonObstacle([(81.03552047673315, 9.875092126168319), (69.74265799914309, 12.014706686432378), (74.90542965144485, 18.63474961713208), (83.42475411346906, 17.178827334611448), (81.03552047673315, 9.875092126168319)])
        #field.createPolygonObstacle([(8.864360059258631, 13.847596946787208), (6.834308465792152, 24.93900329166745), (14.658581078498962, 23.32477083923183), (22.78875670213036, 19.237918739041824), (8.864360059258631, 13.847596946787208)])
        #field.createPolygonObstacle([(28.950174303359844, 14.072547882833728), (20.062610595445037, 22.364452501660796), (32.82810190681889, 29.83362309596624),(34.293967837786596, 15.757182957568974), (28.950174303359844, 14.072547882833728)])
        #field.createPolygonObstacle([(101.46292961705844, 59.142218929188964), (100.73547491697053, 60.03534710466407), (117.12782738773598, 72.08714692777797), (117.63567942390151, 62.034124803542056), (101.46292961705844, 59.142218929188964)])
        #field.createPolygonObstacle([(88.64393658124915, 55.693742624096856), (84.28815919609447, 55.76878082918817), (79.53048070052732, 64.83869851565323), (92.19740506935673, 69.98984300913754), (94.86569197885575, 65.35923302131646), (88.64393658124915, 55.693742624096856)])
    field.plotField()
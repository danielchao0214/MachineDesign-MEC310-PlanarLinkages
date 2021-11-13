# To model the various shapes, lines, and points involved in the design process, the shapely Python project was used.
import math
from shapely import geometry
from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely import affinity

# This function determines whether a point is inside a polygon. It takes into account if the point lies on the boundary or is a vertex.
def in_polygon(point, polygon):
    return polygon.contains(Point(point)) or point in list(zip(*polygon.exterior.coords.xy)) or geometry.LineString(list(zip(*polygon.exterior.coords.xy))).contains(Point(point))

# This function converts a point in LRCS of scoop into coordinates in the GCS given the angle and offset of the scoop.
def LRCStoGCS(LRCSpoint, angle, offset):
    point = list(affinity.translate(affinity.rotate(Point(LRCSpoint), angle, Point(0,0)), offset[0], offset[1]).coords)[0]
    return [point[0], point[1]]

# This function returns the distance between two points
def dist(pt1, pt2):
    return math.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)

# This function finds fixed pivots corresponding to three moving pivots(GCS coords) using three-point analytical synthesis
def findPivot(vectorC1, vectorC2, vectorC3):
    vectorH2 = [vectorC1[0]-vectorC2[0], vectorC1[1]-vectorC2[1]]
    vectorH3 = [vectorC1[0]-vectorC3[0], vectorC1[1]-vectorC3[1]]
    H2x = vectorH2[0]
    H2y = vectorH2[1]
    H3x = vectorH3[0]
    H3y = vectorH3[1]
    F2 = 0.5 * (vectorC1[0]*vectorC1[0]+vectorC1[1]*vectorC1[1] - vectorC2[0]*vectorC2[0]-vectorC2[1]*vectorC2[1])
    F3 = 0.5 * (vectorC1[0]*vectorC1[0]+vectorC1[1]*vectorC1[1] - vectorC3[0]*vectorC3[0]-vectorC3[1]*vectorC3[1])
    Gx = (F3-((H3y/H2y)*F2))/(H3x-(H3y*H2x/H2y))
    Gy = (F2-H2x*Gx)/H2y
    return [Gx, Gy]

# GLOBAL VARIABLES
LRCScoords = []
maxtransmissionangle = 0

# CONSTANTS
# scoop in intial position
E1points = [(0,0),(0,3.48),(0.90,3.50),(1.37,1.49),(4.20,0)]
E1 = Polygon([[0,0],[0,3.48],[0.90,3.50],[1.37,1.49],[4.20,0]])
E1line = geometry.LineString(E1points)

# rectangular obstacle
Rpoints = [(6.48,0),(6.48,1.70),(10.28,1.70),(10.28,0)]
R = Polygon(Rpoints)
Rline = geometry.LineString(Rpoints)

# scoop in final position
angle3 = -36
offset3 = [6.9,4.7]
E3 = affinity.translate(affinity.rotate(E1, angle3, Point(0,0)),offset3[0],offset3[1])
E3points = list(zip(*E3.exterior.coords.xy))
E3line = geometry.LineString(E3points)

# coordinates of the ground
groundpoints = [(-100, -0.1), (100, -0.1)]
ground = geometry.LineString(groundpoints)

# find moving pivots in locally rotating coordinate system (precalculated for runtime)
for LRCSx in range(0, 16):
    LRCSx = LRCSx/4
    for LRCSy in range(0, 14):
        LRCSy = LRCSy/4
        if(not in_polygon([LRCSx, LRCSy], E1)):
            break
        LRCScoords.append([LRCSx, LRCSy])

# find angle/offsets that give shape that does not intersect with ground or rectangle
for angle2 in range(0, 18): # 90/5
    angle2 = angle3 + angle2 * 2
    for o2x in range(1, 7):
        for o2y in range(1, 7):
            offset2 = [o2x, o2y]
            E2 = affinity.translate(affinity.rotate(E1, angle2, Point(0,0)), offset2[0], offset2[1])
            E2points = list(zip(*E2.exterior.coords.xy))
            E2line = geometry.LineString(E2points)
            if(not (E2.intersection(R).is_empty)):
                break
            if(not (E2.intersection(ground).is_empty)):
                break                   

# find moving pivots in global coordinate system
            GCS1 = []
            GCS2 = []
            GCS3 = []
            for coord in LRCScoords:
                GCS1.append(coord)
                GCS2.append(LRCStoGCS(coord, angle2, offset2))
                GCS3.append(LRCStoGCS(coord, angle3, offset3))


# find fixed pivot given 3 positions of moving pivot
            pivots = []
            i = 0
            while i < len(GCS1):
                p = findPivot(GCS1[i], GCS2[i], GCS3[i])
                # if the coordinates are too far, skip
                if(p[1] < 0):
                    GCS1.pop(i)
                    GCS2.pop(i)
                    GCS3.pop(i)
                    continue
                elif(p[0] > 10 or p[1] > 10):
                    GCS1.pop(i)
                    GCS2.pop(i)
                    GCS3.pop(i)
                    continue
                else:
                    i += 1
                    pivots.append(p)

            for i in range(0, len(GCS1)):
                for j in range(i+1, len(GCS1)):
                    moving_pivot1 = GCS1[i]
                    moving_pivot2 = GCS1[j]
                    aa = GCS2[i]
                    ab = GCS2[j]
                    ac = GCS3[i]
                    ad = GCS3[j]
                    fixed_pivot1 = pivots[i]
                    fixed_pivot2 = pivots[j]
                    line1 = geometry.LineString([Point(fixed_pivot1), Point(moving_pivot1)])
                    line2 = geometry.LineString([Point(fixed_pivot2), Point(moving_pivot2)])
                    # if it intersects with the rectangle, skip
                    if(line1.intersects(R) or line2.intersects(R)):
                        continue
                    O2C2 = geometry.LineString([Point(fixed_pivot1), Point(aa)])
                    O4D2 = geometry.LineString([Point(fixed_pivot2), Point(ab)])
                    if(O2C2.intersects(O4D2)):
                        continue
                    a = dist(fixed_pivot1, moving_pivot1)
                    b = dist(moving_pivot1, moving_pivot2)
                    c = dist(moving_pivot2, fixed_pivot2)
                    d = dist(fixed_pivot1, fixed_pivot2)
                    s = min(a, b, c, d)
                    l = max(a, b, c, d)
                    if((l + s) > (a+b+c+d-l-s)):
                        continue
                    if(s < 0.5):
                        continue
                    else:
                        if((b**2+c**2-(d-a)**2)/(2*b*c) > 1):
                            continue
                        mu1 = math.acos((b**2+c**2-(d-a)**2)/(2*b*c))
                        mu2 = math.acos((b**2+c**2-(d+a)**2)/(2*b*c))
                        if(mu1 > math.pi/2):
                            mu1 = math.pi - mu1
                        if(mu2 > math.pi/2):
                            mu2 = math.pi - mu2
                        mu = min(mu1, mu2)
                        if(mu > maxtransmissionangle):
                            maxtransmissionangle = mu
                            print("ANGLE2: ", angle2)
                            print("OFFSET2: ", offset2)
                            print("A1: ", moving_pivot1)
                            print("B1: ", moving_pivot2)
                            print("A2: ", aa)
                            print("B2: ", ab)
                            print("A3: ", ac)
                            print("B3: ", ad)
                            print("mu1: ",  mu1*180/math.pi)
                            print("mu2: ", mu2*180/math.pi)
                            print("SIDE LENGTHS: ", a, b, c, d)
                            print("FP1: ", fixed_pivot1)
                            print("FP2: ", fixed_pivot2)
                            print(math.degrees(mu))
                            print()
                            print()
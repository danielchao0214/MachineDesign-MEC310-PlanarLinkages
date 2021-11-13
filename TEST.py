import math
import numpy as np
from shapely import geometry
from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely import affinity
import pprint

def in_polygon(point, polygon):
    return polygon.contains(Point(point)) or point in list(zip(*polygon.exterior.coords.xy)) or geometry.LineString(list(zip(*polygon.exterior.coords.xy))).contains(Point(point))

def LRCStoGCS(LRCSpoint, angle, offset):
    point = list(affinity.translate(affinity.rotate(Point(LRCSpoint), angle, Point(0,0)), offset[0], offset[1]).coords)[0]
    return [point[0], point[1]]

def dist(pt1, pt2):
    return math.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)

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

LRCScoords = []
master = []
maxtransmissionangle = 0

# CONSTANTS
E1points = [(0,0),(0,3.48),(0.90,3.50),(1.37,1.49),(4.20,0)]
E1 = Polygon([[0,0],[0,3.48],[0.90,3.50],[1.37,1.49],[4.20,0]])
E1line = geometry.LineString(E1points)

Rpoints = [(6.48,0),(6.48,1.70),(10.28,1.70),(10.28,0)]
R = Polygon(Rpoints)
Rline = geometry.LineString(Rpoints)

angle3 = 29
offset3 = [5,4]
E3 = affinity.translate(affinity.rotate(E1, angle3, Point(0,0)),offset3[0],offset3[1])
E3points = list(zip(*E3.exterior.coords.xy))
E3line = geometry.LineString(E3points)

groundpoints = [(-100, -0.1), (100, -0.1)]
ground = geometry.LineString(groundpoints)

print(E3points)
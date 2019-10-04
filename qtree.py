# -*- coding: utf-8 -*-
"""
Created on Thu Oct  3 14:29:45 2019

@author: Sven van Koutrik
"""
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class Point:
    def __init__(self, x=0, y=0, ID=None, parent=None):
        self.x = x;
        self.y = y;
        self.parent = parent;
        self.ID = ID;

    def dist_points(self,pt):
        return (self.x-pt.x)**2 + (self.y-pt.y)**2

        
class Node():
    #a quadtree node is a box
    def __init__(self, x0, y0, w, h, points, parent):
        self.x0 = x0
        self.y0 = y0
        self.width = w
        self.height = h
        self.points = points
        self.children = []
        self.parent = parent;
    
class QTree():
    def __init__(self, k, x0, y0, w, h):
        self.threshold = k #max nb of points per box
        self.points = []
        self.root = Node(x0,y0, w, h, [], None)

    def add_point(self, point):
        
        node = recursive_find_node(self.root,point)
        self.points.append(point)
        node.points.append(point)
        recursive_subdivide(node, self.threshold)
    
    def nearest_point(self, point):
        
        #find node of target point and nearest point inside this node
        node=recursive_find_node(self.root,point)
        while not node.points:
            node=node.parent
        nearest_prelim = nearest_point_in_nodes([node],point)            
                        
        #make box around target point (ideal is a circle)
        dx = abs(nearest_prelim.x - point.x)
        dy = abs(nearest_prelim.y - point.y)
        xmin,xmax = point.x-dx, point.x+dx
        ymin,ymax = point.y-dy, point.y+dy
        
        #find nodes that interesect with box
        nodes = recursive_box_intersect(list(), self.root, xmin, xmax, ymin, ymax)
        
        #check all points in all these nodes
        nearest = nearest_point_in_nodes(nodes,point)
            
        return nearest

    def subdivide(self):
        recursive_subdivide(self.root, self.threshold)
    
    def graph(self):
        fig = plt.figure(figsize=(12, 8))
        plt.title("Quadtree")
        ax = fig.add_subplot(111)
        c = find_children(self.root)
        print ("Number of segments: %d" %len(c))
        areas = set()
        for el in c:
            areas.add(el.width*el.height)
        print ("Minimum segment area: %.3f units" %min(areas))
        for n in c:
            ax.add_patch(patches.Rectangle((n.x0, n.y0), n.width, n.height, fill=False))
        x = [point.x for point in self.points]
        y = [point.y for point in self.points]
        plt.plot(x, y, 'ro')
        plt.show()
        return


def nearest_point_in_nodes(nodes,point):
    
    minSqDist = math.inf
    for node in nodes:
        for pt in node.points:
            sqDist = point.dist_points(pt)
            if sqDist < minSqDist:
                minSqDist = sqDist
                nearest = pt
    return nearest
   

def recursive_box_intersect(nodes, node, xmin, xmax, ymin, ymax):
    
    xOverlap, yOverlap = False,False
    if (xmin>=node.x0 and xmin <= node.x0+node.width) or (xmax>=node.x0 and xmax <= node.x0+node.width):
        xOverlap = True
    if (ymin>=node.y0 and ymin <= node.y0+node.height) or (ymax>=node.y0 and ymax <= node.y0+node.height):
        yOverlap = True   
        
    if not (xOverlap and yOverlap): #return if no overlap
        return nodes
    
    if not node.children:
        nodes.append(node)
        return nodes
    
    for nd in node.children:    
        nodes = recursive_box_intersect(nodes, nd , xmin, xmax, ymin, ymax)    
    return nodes



def recursive_find_node(node,point):
    if not node.children:
        return node
    
    isLeft = point.x < node.x0 + node.width/2 
    isLow  = point.y < node.y0 + node.height/2
    if isLeft and isLow:
        child=0
    if isLeft and not isLow:
        child=1
    if not isLeft and isLow:
        child=2
    if not isLeft and not isLow:
        child=3
    return recursive_find_node(node.children[child],point)



def circle_box_collision(node,point,r):
    #not used (instead box-box collision is used)
    
    #closest point of rectangle to circle center
    xc = clamp(point.x, node.x0, node.x0+node.width)
    yc = clamp(point.y,node.y0,node.y0+node.height)
    (xc-point.x)**2+(yc-point.y)**2 < r**2
    #check corner
    if (xc-point.x)**2+(yc-point.y)**2 < r**2: 
        return True #corner collision
    else:
        return False

def clamp(val,upper,lower):
    return max(min(val,lower),upper)



def recursive_subdivide(node, k):
    if len(node.points)<=k:
        return
    
    w_ = float(node.width/2)
    h_ = float(node.height/2)
    
    #figure out which subdivisions all original points go
    p = contains(node.x0, node.y0, w_, h_, node.points)
    x1 = Node(node.x0, node.y0, w_, h_, p, node)
    recursive_subdivide(x1, k)
    
    p = contains(node.x0, node.y0+h_, w_, h_, node.points)
    x2 = Node(node.x0, node.y0+h_, w_, h_, p, node)
    recursive_subdivide(x2, k)
    
    p = contains(node.x0+w_, node.y0, w_, h_, node.points)
    x3 = Node(node.x0 + w_, node.y0, w_, h_, p, node)
    recursive_subdivide(x3, k)
    
    p = contains(node.x0+w_, node.y0+w_, w_, h_, node.points)
    x4 = Node(node.x0+w_, node.y0+h_, w_, h_, p, node)
    recursive_subdivide(x4, k)
    
    node.children = [x1, x2, x3, x4]

    
def contains(x, y, w, h, points):
    pts = []
    for point in points:
        if point.x >= x and point.x <= x+w and point.y>=y and point.y<=y+h:
            pts.append(point)
    return pts


def find_children(node):
    if not node.children:
        return [node]
    else:
        children = []
        for child in node.children:
            children += (find_children(child))
    return children



if __name__ == "__main__":
    #testing
    
    import random
    x0,y0,w,h = 0,0,10,10
    k,n = 5,1000
    qt = QTree(k, x0,y0,w,h)
    for i in range(n):
        qt.add_point(Point(random.uniform(x0, x0+w),random.uniform(y0, y0+h)))        
    qt.graph()
    
    import time
    t0 = time.clock()
    qt.add_point(Point(5.5,6.5))
    t1 = time.clock()
    total = t1-t0
    print(total)
    
    
    t0 = time.clock()
    pt = qt.nearest_point( Point(5.1,4.8) )
    t1 = time.clock()
    total = t1-t0
    print(total)
    
    print(pt.x)
    print(pt.y)





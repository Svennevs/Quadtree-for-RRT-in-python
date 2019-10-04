# -*- coding: utf-8 -*-
"""
Created on Mon Sep 30 17:50:12 2019

@author: Sven van Koutrik
"""
import random, math, csv
from qtree import QTree, Point

class Edge:
     def __init__(self, n1=None, n2=None, cost=0):
         self.ID1 = n1.ID;
         self.ID2 = n2.ID;
         self.cost = math.sqrt( n1.dist_points(n2) );

class Obstacle:
    def __init__(self, x=0, y=0, d=0):
        self.x = x;
        self.y = y;
        self.r = d/2;
        
    def pointOnObstacle(self,point):
        return (point.x-self.x)**2 + (point.y-self.y)**2  <= self.r**2
        
    def LineOnObstacle(self,point1,point2):
        
        #calc equivalent system with obstacle center 0,0
        #solve line-circle intersection first, afterwards check line segment        
        x1 = point1.x - self.x;
        x2 = point2.x - self.x;
        y1 = point1.y - self.y;
        y2 = point2.y - self.y;
        
        dx  = x2-x1;
        dy  = y2-y1;
        drs = dx**2 + dy**2;
        D   = x1*y2 - x2*y1;
        discriminant = self.r**2*drs- D**2
        
        if discriminant < 0: 
            return False;
        else:#intersection of line. Check if the segment intersects too.
            #both points are not in obstacles, so either both intersections are 
            #on the segment, or none. Only need to check 1.
            xs1 = (D*dy + dx*math.sqrt(discriminant))/drs;

            #check if x-co of intersection is between x-co of points
            if (xs1 >= x1 and xs1 <= x2)  or (xs1 >= x2 and xs1 <= x1):
                return True
            else:
                return False



def SampleRRTNode():
    #takes uniform random samples in c-space until a point that not
    #lies on an obstacle is found
    free = False
    while not free: 
        RRTnode = Point( random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5) )
        free = True #free unless proven otherwise in loop below
        for obst in obstacles: #loop through obstacles
            if obst.pointOnObstacle(RRTnode):
                free = False;
    return RRTnode

def loadObstacles(path_obstacles):
    #loads obstacle data into obstacle objects
    obstacles=[];
    with open(path_obstacles) as fd:
        for n, ln in enumerate(fd):
            if ln[0] != '#': #if no comment
                ln =  ln.split(',');
                obstacles.append(Obstacle(float(ln[0]),float(ln[1]),float(ln[2])));
    return obstacles

def setStep(RRTnode1,RRTnode2):
    #makes new RRTnode that takes stepSize length step from RRTnode1 to RRTnode2
    #step should never be longer than the distance between RRTnode1 and RRTnode2
    stepFraction = min([1,stepSize / math.sqrt( RRTnode1.dist_points(RRTnode2) )])

    xn = RRTnode1.x + (RRTnode2.x - RRTnode1.x)*stepFraction
    yn = RRTnode1.y + (RRTnode2.y - RRTnode1.y)*stepFraction
    RRTnodeNew = Point(xn,yn)
    return RRTnodeNew
    
def LocalPlanner(RRTnode1,RRTnode2):
    #see if straight line between RRTnodes is obstacle free
  
    for obst in obstacles:
        if obst.LineOnObstacle(RRTnode1,RRTnode2):
            return False
    return True



if __name__ == "__main__":
    
    #As tree, a quadtree object is used.It stores the RRT nodes as quadtree points
    # (because quadtree nodes are boxes)
    
    path_obstacles = './files/obstacles.csv'
    path_edges     = './files/edges.csv'
    path_RRTnodes  = './files/nodes.csv'
    path_path      = './files/path.csv'
    
    maxTreeLength   = 1000;
    goalTryInterval = 10; #iterations between trying to connect goal RRTnode 
    stepSize        = 0.3;
    xmin, xmax      = -0.5, 0.5
    ymin, ymax      = -0.5, 0.5
    start           = Point(xmin, ymin, 1)
    goal            = Point(xmax, ymax)
    
    obstacles = loadObstacles(path_obstacles) # list with obstacle objects
    
    T         = QTree(10, xmin, ymin, xmax-xmin, ymax-ymin ); # Tree
    T.add_point(start)
    edges     = []       # List of edges
    path      = []       # List of resulting path
    success   = False
    
    ctr = 0
    while len(T.points) < maxTreeLength: #RRT loop
        ctr+=1 #increase counter
    
        if ctr % goalTryInterval:  #check division remainder
            xSamp = SampleRRTNode() #makes sure new RRTnode is not on obstacle
        else:
            xSamp = goal #every now and then try the goal RRTnode
            
        xNearest = T.nearest_point( xSamp )
        xNew = setStep(xNearest,xSamp)
        
        if LocalPlanner(xNearest,xNew): #checks for collision-free path
            xNew.parent = xNearest; # set parent of xSamp to nearest
            xNew.ID = len(T.points)+1;     # give RRTnode an ID
            T.add_point(xNew);         # add to tree
            edges.append(Edge(xNearest,xNew)) #generate edge
            
            if xNew.x == goal.x and xNew.y == goal.y:
                T.graph()    

                success = True
                while xNew: #reconstruct path backwards from goal to start
                    path.insert(0,xNew.ID);
                    xNew = xNew.parent;
                break#success
        else: #no collision-free path found
            continue
    
    #postprocess
    if success:
        with open(path_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(path)
        
        with open(path_RRTnodes, 'w', newline='') as f:
            w = csv.writer(f)
            
            for RRTnode in T.points:
                w.writerow([RRTnode.ID, RRTnode.x, RRTnode.y, 0])
    
        with open(path_edges, 'w', newline='') as f:
            w = csv.writer(f)
            for edge in edges:
                w.writerow([edge.ID1, edge.ID2, edge.cost]) 
    else:
        print('no path found')
        
        




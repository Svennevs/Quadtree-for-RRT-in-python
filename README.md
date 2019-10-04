# Quadtree-for-RRT-in-python

##summary
Extended the quadtree implementation from [https://github.com/kpully/quadtree](https://github.com/kpully/quadtree). Included nearest neighbor search.
- finds node corresponding to point and nn inside node
- draws a box around the point (size dep on distance to initial nn)
- rectangle collision gives all node canditates for global nn.

##application
the quadtree and node class are used for an RRT program made for a coursera Modern Robotics (course 4, week 2) assignment [link](https://www.coursera.org/specializations/modernrobotics)


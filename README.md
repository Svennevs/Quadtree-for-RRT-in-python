# Quadtree for RRT in python

## summary
Quadtree implementation from [https://github.com/kpully/quadtree](https://github.com/kpully/quadtree) extended with nearest neighbor search.
* finds node corresponding to point and local nn (inside node)
* draws a box around the point (size dep on distance to local nn)
* rectangle collision gives all node canditates for global nn.

## application
The quadtree and node class are used for an RRT program made for a [coursera Modern Robotics](https://www.coursera.org/specializations/modernrobotics)(course 4, week 2) assignment [link](http://hades.mech.northwestern.edu/index.php/Sampling-Based_Planning).


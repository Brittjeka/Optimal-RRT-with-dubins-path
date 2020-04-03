# Robot motion planning and control - group 10 - car - RRT 
Implementation of a RRT* algorithm for a car

### `Main.m` 
File with the main code of RRT* star. To run the code download all files and run the `main.m` file 
For changing the different scenario's, change the value of the variable `scenario` at the top
of the main.m file to `1`,`2` or `3`

### `BuildObstacles.m`
Implementation of the environment with the obstacles. 

### `dist.m`
Function for calculating the distance between two points in the x,y plane

### `dubins_core.m` and `dubins_curve.m`
Functions written by Kang (2020) are implemented in the `main.m` file to calculate the 
dubins path between `q_new` and `q_near(est)`

### `InitiatePlot.m`
File that creates the plot with all the obstacles and the orientation/location of the start and the goal

###  `InitiateVars.m`
All the initial paramters and values

### `noCollisionDubinspath.m`
Function which is based on the method of Vemprala (2020), however modified for the dubinspath.
The function prevents the collision of the nodes and connected lines with the obstacles 

### `ccw.m`
Function which is used in `NoCollisionDubinspath.m`

### `steer.m`
Function with the steering function to calculate `q_new` when `q_rand` and `q_near` are given.
Originally written by Vemprala (2020) and modified for our situations.

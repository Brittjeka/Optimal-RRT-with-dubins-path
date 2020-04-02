% Define map
x_max = 3000;
y_max = 2000;
theta_max = pi;  % for generating random nodes, theta_min = -pi

% Variable for dubins path
stepsize = 1; 

% start function
% [q_start is a structure array with coord [x,y,theta] (1x3),
% cost(scalar) and parent (scalar)]
q_start.cost = 0;
q_start.parent = 0;

% [node is a structure array that contains coord, cost and parent of all
% nodes who are used to reach the goal. Node is a 1x'totalnumberofnodes'
% strcture with 3 fields. The first row is added, that is the
% startposition q_start]
nodes(1) = q_start;

% Build obstacles in map
obstacles = buildObstacles(scenario);

% Make boundary around obstacles so the car cannot hit them in any
% orientation
col_obstacles = obstacles + [-R,-R,2*R,2*R].*ones(size(obstacles,1),4);

dist_temp = inf; %Initial value for the distance between the new node and the goal coordinate (will be updated inside the loop)

% Tracking variables
counter = 0;
expandingTree = 1;
checkpoint1 = 0;
checkpoint2 = 0;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference:
%   SAI VEMPRALA (2020). 2D/3D RRT* algorithm 
%   (https://www.mathworks.com/matlabcentral/fileexchange/60993-2d-3d-rrt-algorithm), 
%   MATLAB Central File Exchange. Retrieved February 24, 2020.
%
% The function 'main.m', is based on the method used by SAI VEMPRALA
% (2020). The function is modified to obtain a RRT* model, which simulates
% a car driving in the city center of Delft from point A to point B. 
% The model makes sure the car automatically chooses the shortest path from 
% the current location of the car to the destination.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars
close all
clear all

% Setup
EPS = 150;                              % maximum distance new node and old nodes
r = 70;                                 % turning radius car
r_rrt = 500;                            % search for better nodes in this range
W = 70;                                 % Width of car
L = 100;                                % Length of car
R = sqrt((L/2)^2+(W/2)^2);              % Worst case radius
q_start.coord = [50+R 50+R -0.5*pi];    % Starting coordinate
scenario = 1;                           % Choose scenario
extraNodes1 = 75;                       % #nodes until improved path is drawn
extraNodes2 = 150;                      % #nodes until 2nd and final improved path  is drawn
branchingNodes = 10;                    % #nodes that are checked to branch out of

% Define goal
% [q_goal is a structure array with coord [x,y,theta] (1x3),
% cost(scalar) and parent (scalar), once it has a path]
if scenario == 1
    q_goal.coord = [2600 1600 -0.5*pi];
    q_goal.cost = 0;
elseif scenario == 2 
    q_goal.coord = [2400 1450 pi];
    q_goal.cost = 0;
elseif scenario == 3
    q_goal.coord = [2000 1500 pi];
    q_goal.cost = 0;
end

% Goal region
if scenario == 3
    threshold = 250; %Threshold to be into the goal
else
    threshold = 80; %Threshold to be into the goal
end

% Initiate standard variables and build plot 
initiateVars
initiatePlot

% Main loop, will run until a path from the start node till the end node is
% found, then keeps running for a couple more nodes (user defined) to find 
% more optimal paths 
while expandingTree
    % Once a solution is found, count the additional nodes 
    if checkpoint1 == 1
        counter = counter + 1
    end
    
    % [q_rand (1x3): een random node wordt gekozen, floor zorgt ervoor dat de x en y
    % coordinaten een integer zijn, rand(1) geeft een random scalar tussen
    % 0 en 1]
    q_rand.coord = [floor(rand(1)*x_max) floor(rand(1)*y_max) (-1+2*rand(1))*theta_max];
    
    % Once we are in the goal region, add the goal node to the tree
    if checkpoint1 == 1 && counter == 1
        q_rand.coord = q_goal.coord;
    end

    % Plot an x where q_rand is generated
    plot(q_rand.coord(1), q_rand.coord(2), 'x', 'Color',  [0 0.4470 0.7410])
   
    % Find nearby nodes from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand.coord);
        ndist = [ndist tmp]; % contains the Euclidian distance of q_rand to all nodes
    end
    
    % Define amount of nodes to branch out of, which normally is equal to
    % the variable branchingNodes, except if there are not enough nodes or
    % if we try to find a path to the goal region, in the latter case the
    % algorithm will try all nodes to make sure none are missed
    if length(nodes)<=branchingNodes
        max_cnt = length(nodes); 
    elseif q_rand.coord == q_goal.coord
        max_cnt = length(nodes);
    else
        max_cnt = branchingNodes;
    end
    
    % Select nearest nodes
    [vals, idxs] = mink(ndist,max_cnt);
    
    % This loop tries to find a node to branch out of, starting with the
    % node that is closest, assume there is no path possible
    available_path = 0;
    for cnt = 1:max_cnt
        % Select node, call it q_near 
        val = vals(cnt);
        idx = idxs(cnt);
        q_nearest = nodes(idx);
        
        % Branch out to q_new, which is in the direction of q_rand, but
        % takes the max branch range into account. All coordinates are
        % scaled accordingly 
        q_new.coord = steer(q_rand.coord, q_nearest.coord, val, EPS);
        
        % If the coordinate in question is the goal node, ignore the
        % branching length
        if checkpoint1 == 1 && counter == 1
            q_new.coord = q_goal.coord;
        end

        % Calculate dubins path
        param = dubins_core(q_new.coord, q_nearest.coord, r);
        path = dubins_curve(q_new.coord, q_nearest.coord, r, stepsize);
        
        % Check for collisions with obstacles, if not, then a path is
        % available
        if noCollisionDubinspath([path;q_nearest.coord], col_obstacles)
            available_path = 1;
            break
        end
    end
    
    if available_path == 1        
        % Plot the found Dubins path
        plot([path(:,1);q_nearest.coord(:,1)], [path(:,2);q_nearest.coord(:,2)], 'Color', 'k', 'LineWidth', 2);
        drawnow
        hold on
        
        % Calculate the cost to reach the new node
        q_new.cost = r * (param.seg_param(1)+param.seg_param(2)+param.seg_param(3)) + q_nearest.cost;
        
        % Initialize cost to currently known value
        q_min = q_nearest;     % Initial cheapest node to connect to
        C_min = q_new.cost; % Initial cost to reach that node
        param_min = param;  % Initial parameters of Dubins path
        path_min = [path;q_min.coord]; % Initial minimal path
        q_near = [];
        
        % Check if euclidean distance < r_rrt for every node, save these 
        % nodes
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if dist(nodes(j).coord, q_new.coord) <= r_rrt
                q_near(neighbor_count).coord = nodes(j).coord;
                q_near(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
       
        % For each of the nearby node, check whether the dubins path is in collision
        % choose the shortest path of all paths that are not in collision
        for k = 1:1:length(q_near)
            q_n = q_near(k);
            param = dubins_core(q_new.coord, q_n.coord, r);
            path = dubins_curve(q_new.coord, q_n.coord, r, stepsize);
            % Check collision
            if noCollisionDubinspath([path;q_n.coord], col_obstacles)
                % Check if cheaper
                if q_near(k).cost + r * (param.seg_param(1)+param.seg_param(2)+param.seg_param(3))< C_min
                    % Update path to cheaper node
                    q_min = q_near(k);
                    C_min = q_near(k).cost + r * (param.seg_param(1)+param.seg_param(2)+param.seg_param(3));
                    param_min = param;
                    path_min = [path;q_min.coord];
                    hold on
                end
            end
        end
        
        % Draw line of new path    
        figure(1)
        plot(path_min(:,1), path_min(:,2), 'Color', 'g', 'LineWidth', 0.8);%    
        
        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        
        % Save minimal cost found
        q_new.cost = C_min; 
        
        % Append new node to nodes
        nodes = [nodes q_new];
        
        % Rewire nodes, for nodes in range of r_rrt is checked whether the
        % path to the new node is cheaper. If this is the case, the path is
        % changed and the costs are updated of the whole chain
        for k = 1:1:length(q_near)
            q_n = q_near(k);
            param = dubins_core(q_n.coord, q_new.coord, r);
            path = dubins_curve(q_n.coord, q_new.coord, r, stepsize);
            % Check for collision
            if noCollisionDubinspath([path;q_new.coord], col_obstacles) 
                % Check if cheaper
                if q_new.cost + r * (param.seg_param(1)+param.seg_param(2)+param.seg_param(3))<q_n.cost
                    for j = 1:1:length(nodes)
                        % If it is cheaper set new node as parent node
                        if nodes(j).coord == q_n.coord
                            nodes(j).parent = length(nodes);
                            diff_cost = nodes(j).cost - (q_new.cost + r * (param.seg_param(1)+param.seg_param(2)+param.seg_param(3)));
                            nodes(j).cost = q_new.cost + r * (param.seg_param(1)+param.seg_param(2)+param.seg_param(3));
                            notUpdated = 1;
                            search_parent = j;
                            % Update the costs of the whole chain
                            while notUpdated
                                for iter = 1:length(nodes)
                                    if nodes(iter).parent == search_parent(1)
                                        nodes(iter).cost = nodes(iter).cost - diff_cost;
                                        search_parent = [search_parent iter];
                                    end
                                end
                                search_parent(1) = [];
                                if isempty(search_parent)
                                    notUpdated = 0;
                                end
                            end
                        end
                    end
                end               
            end
        end
        
        distance_to_goal = dist(q_new.coord,q_goal.coord); 
        
        % Check if the node is in the goal region 
        if distance_to_goal <= threshold && checkpoint1 == 0
            checkpoint1 = 1;
        end
        
        % After the goal node is added to the nodes, backtrack to find the
        % path
        if checkpoint1 == 1 & q_new.coord == q_goal.coord
            disp('checkpoint 1')
            q_end = nodes(end);
            q_end_save1 = q_end;
            q_end_index = length(nodes);
            first_path = [];
            % Backtracking
            while q_end.parent ~= 0
                start = q_end.parent;
                path1 = dubins_curve(q_end.coord, nodes(start).coord, r, stepsize); %Compute dubinscurve for the most optimal path
                first_path = [first_path; path1];
                plot(path1(:,1),path1(:,2), 'Color', 'r', 'LineWidth', 3) %Plot path
                hold on
                q_end = nodes(start);
            end
        end
        

    end
    % After some extra nodes, find the optimal path
    if counter == extraNodes1
        disp('checkpoint2')
        second_path = [];
        q_end = nodes(q_end_index);
        nodes2 = nodes;
        q_end_save2 = q_end;
        % Backtracking to find the optimal path
        while q_end.parent ~= 0
            start = q_end.parent;
            path2 = dubins_curve(q_end.coord, nodes(start).coord, r, stepsize); %Compute dubinscurve for the most optimal path
            second_path = [second_path; path2];
            plot(path2(:,1),path2(:,2), 'Color', 'r','LineStyle','--', 'LineWidth', 3) %Plot path
            hold on
            q_end = nodes(start);
        end
    end
    
    % After some extra nodes, find the optimal path and stop the loop
    if counter == extraNodes2
        disp('checkpoint3')
        third_path = [];
        q_end = nodes(q_end_index);
        q_end_save3 = q_end;
        % Backtracking to find the optimal path
        while q_end.parent ~= 0
            start = q_end.parent;
            path3 = dubins_curve(q_end.coord, nodes(start).coord, r, stepsize); %Compute dubinscurve for the most optimal path
            third_path = [third_path; path3];
            plot(path3(:,1),path3(:,2), 'Color', 'r','LineStyle','-.', 'LineWidth', 3) %Plot path
            hold on
            q_end = nodes(start);
        end
        % Stop searching for new nodes
        expandingTree = 0;
    end
end

% Cost of paths
cost_path1 = q_end_save1.cost 
cost_path2 = q_end_save2.cost 
cost_path3 = q_end_save3.cost 
%% Animation

% This part runs an animation of the car along the optimal path
figure(2)
end_path = third_path;
for i = size(end_path,1):-5:1
    % Set axis
    axis([-100 x_max+100 -100 y_max+100])
    % Build opstacles
    for j = 1:size(obstacles,1)
        rectangle('Position',obstacles(j,:),'FaceColor',[0 .5 .5])
        hold on
    end
    %plot start position 
    x0      = q_start.coord(1); 
    y0      = q_start.coord(2); 
    theta0  = q_start.coord(3); 
    plot(x0,y0,'ko', 'MarkerSize',5,'MarkerFaceColor','k')
    ed     = [cos(theta0) -sin(theta0); sin(theta0) cos(theta0)]*[20*1.5;0];
    plot([x0,x0+ed(1)],[y0,y0+ed(2)],'y-', 'Linewidth',6);
    plot(x0+ed(1),y0+ed(2),'ko','MarkerSize',5, 'MarkerFaceColor','b'); 
    plot(end_path(:,1),end_path(:,2),'r--', 'LineWidth', 2) %Plot path
    x = end_path(i,1);
    y = end_path(i,2);
    theta = end_path(i,3)/pi*180;
    % Plot car
    car_position = polyshape([x+L/2,x-L/2, x-L/2. x+L/2],[y-W/2,y-W/2, y+W/2. y+W/2]);
    poly2 = rotate(car_position,theta,[x,y]);
    plot(poly2)
    axis equal
    axis([-100 x_max+100 -100 y_max+100])
    
    % Update plot
    drawnow
    clf
end


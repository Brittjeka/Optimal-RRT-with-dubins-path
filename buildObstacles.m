function obstacles = buildObstacles(scenario)
% This function contains various scenarios which can be tested. The map has
% a standard size of 3000x2000.
% each row is one obstacle with [x_left, y_bottom, width (x direction), 
% height(y direction)]
if scenario == 1
    obstacles = [300,300,200,200;
                900,0,200,500;
                400,900,600,200;
                200,1400,200,200;
                800,1400,200,200;
                1400,1300,200,700;
                1400,800,200,200;
                1400,300,200,200;
                2000,300,200,500;
                2000,1200,200,500;
                -10,0,10,2000; % left boundary plot
                0,-10,3000,10; % lower boundary plot
                0,2000,3000,10; % top boundary plot
                3000,0,10,2000]; % right boundary plot
elseif scenario == 2
    obstacles = [200,400,200,700;
                200,1300,200,500;
                700,1300,200,500;
                700,400,200,600;
                700,250,600,150;
                1200,1600,500,50;
                1200,1300,500,50;
                1200,1000,500,50;
                1200,700,500,50;
                
                -10,0,10,2000; % left boundary plot
                0,-10,3000,10; % lower boundary plot
                0,2000,3000,10; % top boundary plot
                3000,0,10,2000; % right boundary plot
                2000,1600,600,100;
                2000,1200,600,100;
                2000,800,600,100;
                2000,400,600,100;
                2600,400,200,1300];
elseif scenario == 3   
    obstacles = [-10,0,10,2000; % left boundary plot
                0,-10,3000,10; % lower boundary plot
                0,2000,3000,10; % top boundary plot
                3000,0,10,2000]; % right boundary plot
end

end
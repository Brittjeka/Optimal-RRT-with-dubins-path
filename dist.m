%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference:
%   SAI VEMPRALA (2020). 2D/3D RRT* algorithm 
%   (https://www.mathworks.com/matlabcentral/fileexchange/60993-2d-3d-rrt-algorithm), 
%   MATLAB Central File Exchange. Retrieved February 24, 2020.
%
% The function 'dist.m', written by SAI VEMPRALA (2020), is used in main.m to
% calculate the distance between two points in the x,y plane.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function d = dist(q1,q2)
d = sqrt((q1(1)-q2(1))^2 + (q1(2)-q2(2))^2);
end
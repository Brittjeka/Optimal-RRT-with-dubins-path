%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference:
%   SAI VEMPRALA (2020). 2D/3D RRT* algorithm 
%   (https://www.mathworks.com/matlabcentral/fileexchange/60993-2d-3d-rrt-algorithm), 
%   MATLAB Central File Exchange. Retrieved February 24, 2020.
%
% The function 'noCollisionDubinspath.m', is based on the method used by SAI VEMPRALA (2020)
% in his function 'noCollision.m'.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function nc = noCollisionDubinspath(path,o)
    nc = 1; % Assume no collision
    
    for i = 1:length(path)-1
        for j = 1:size(o,1)
            A = [path(i,1) path(i,2)];
            B = [path(i+1,1) path(i+1,2)];
                obs = [o(j,1) o(j,2) o(j,1)+o(j,3) o(j,2)+o(j,4)];

            C1 = [obs(1),obs(2)];
            D1 = [obs(1),obs(4)];
            C2 = [obs(1),obs(2)];
            D2 = [obs(3),obs(2)];
            C3 = [obs(3),obs(4)];
            D3 = [obs(3),obs(2)];
            C4 = [obs(3),obs(4)];
            D4 = [obs(1),obs(4)];

            % Check if path from n1 to n2 intersects any of the four edges of the
            % obstacle

            ints1 = ccw(A,C1,D1) ~= ccw(B,C1,D1) && ccw(A,B,C1) ~= ccw(A,B,D1); 
            ints2 = ccw(A,C2,D2) ~= ccw(B,C2,D2) && ccw(A,B,C2) ~= ccw(A,B,D2);
            ints3 = ccw(A,C3,D3) ~= ccw(B,C3,D3) && ccw(A,B,C3) ~= ccw(A,B,D3);
            ints4 = ccw(A,C4,D4) ~= ccw(B,C4,D4) && ccw(A,B,C4) ~= ccw(A,B,D4);

            logic = ints1==0 && ints2==0 && ints3==0 && ints4==0;
            if ~logic
                nc = 0;
                break
            end
        end
        if nc == 0
            break
        end
    end
end
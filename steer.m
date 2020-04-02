%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference:
%   SAI VEMPRALA (2020). 2D/3D RRT* algorithm 
%   (https://www.mathworks.com/matlabcentral/fileexchange/60993-2d-3d-rrt-algorithm), 
%   MATLAB Central File Exchange. Retrieved February 24, 2020.
%
% The function 'steer.m', originally written by SAI VEMPRALA (2020), is used in main.m to
% calculate q_new when q_rand and q_near are given. The script is adjusted for our situation.    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function A = steer(qr, qn, val, eps)
   qnew = [0 0 0];
   
   % Steer towards qn with maximum step size of eps
   if val >= eps
       qnew(1) = qn(1) + ((qr(1)-qn(1))*eps)/dist(qr,qn);
       qnew(2) = qn(2) + ((qr(2)-qn(2))*eps)/dist(qr,qn);
       qnew(3) = qr(3) + ((qr(3)-qn(3))*eps)/dist(qr,qn);;
   else
       qnew(1) = qr(1);
       qnew(2) = qr(2);
       qnew(3) = qr(3);
   end   
   A = [qnew(1), qnew(2), qnew(3)];
end
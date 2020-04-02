% Initialize plot

axis([-100 x_max+100 -100 y_max+100])
% Draw all obstacles
for i = 1:size(obstacles,1)
    rectangle('Position',obstacles(i,:),'FaceColor',[0 .5 .5])
    hold on
    rectangle('Position',col_obstacles(i,:),'EdgeColor','r')
end

% Draw start position 
x0      = q_start.coord(1); 
y0      = q_start.coord(2); 
theta0  = q_start.coord(3);
plot(x0,y0,'ko', 'MarkerSize',5,'MarkerFaceColor','k')
ed     = [cos(theta0) -sin(theta0); sin(theta0) cos(theta0)]*[20*1.5;0];
plot([x0,x0+ed(1)],[y0,y0+ed(2)],'y-', 'Linewidth',6);
plot(x0+ed(1),y0+ed(2),'ko','MarkerSize',5, 'MarkerFaceColor','b'); 
 
%Plot goal with threshold using an * for the goal and a circle for the
%treshold region and the yellow line is the orientation
s1      = 4; 
x1      = q_goal.coord(1);
y1      = q_goal.coord(2);
theta1   = q_goal.coord(3); 
t1      = 0:(pi/(5*s1)):2*pi; % s1 is the number of points I want to have on the  circle
x1unit  = threshold * cos(t1) + x1;  
y1unit  = threshold * sin(t1) + y1;
ed     = [cos(theta1) -sin(theta1); sin(theta1) cos(theta1)]*[20*1.5;0];

% plotting end position with * as goal and yellow line as configuration
plot([x1,x1+ed(1)],[y1,y1+ed(2)],'y-', 'Linewidth',6);
plot(x1+ed(1),y1+ed(2),'ko','MarkerSize',5, 'MarkerFaceColor','b'); 
plot(x1unit, y1unit, 'b', x1, y1,'*');%circle  with the center x1,y1
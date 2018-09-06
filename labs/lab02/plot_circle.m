close all;
[object_x,object_y] = find_min_distance(robot);
[circle_x,circle_y,circle_R] = calculate_circle(object_x,object_y);

figure
scatter(0,0,'rx')
%ylim([-1.5*circle_R,1.5*circle_R]);
daspect([1 1 1])
xlim([-3.5*circle_R,3.5*circle_R]);
ylim([-3.5*circle_R,3.5*circle_R]);

hold on
scatter(object_x,object_y,'go')
scatter(circle_x,circle_y,'y*')

theta = linspace(0,2*pi,100);
scatter(circle_R*cos(theta)+circle_x,circle_R*sin(theta)+circle_y, 'b.')
legend('robot','object','circle midpoint','circle path')

xlabel('x (forward direction)')
ylabel('y (left side direction)')
title('Path generation towards object using cicle path')
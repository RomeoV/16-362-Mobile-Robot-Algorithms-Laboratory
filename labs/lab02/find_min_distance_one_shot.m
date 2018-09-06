function [x,y,min_r,min_theta] = find_min_distance_one_shot(robot)
%FIND_MIN_DISTANCE Computes x and y position of closest object (above
%threshhold)

r_values = circshift(robot.laser.LatestMessage.Ranges,-5);
r_values(r_values>2) = nan;
r_values(r_values<0.08) = nan;
r_values(91:270) = nan;
[min_r, min_theta] = min(r_values);
min_theta = min_theta - 1;

y = sin(deg2rad(min_theta))*min_r;
x = cos(deg2rad(min_theta))*min_r;
end


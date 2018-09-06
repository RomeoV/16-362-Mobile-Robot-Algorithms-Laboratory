function [x,y] = find_min_distance_averaged(robot)
%FIND_MIN_DISTANCE Computes x and y position of closest object (above
%threshhold)
%   1) Spin up sensor (takes roughly 2 seconds)
%   1) Measure data over time
%   2) Shift angles and average data
%   4) Remove points that are too close / too far
%   5) Find min radius and angle
%   6) Convert to x and y coordinates

robot.startLaser()
pause(2.5)
tic()
r_matrix = zeros(360,10);
index = 1;
while toc() < 1
    r_matrix(:,index) = circshift(robot.laser.LatestMessage.Ranges,-5);
    index=index+1;
    pause(.1)
end
r_matrix(r_matrix>2) = nan;
r_matrix(r_matrix<0.15) = nan;
r_matrix(isoutlier(r_matrix,2)) = nan;
r_mean = mean(r_matrix,2,'omitnan');
r_mean(91:270) = nan;
[min_r, min_theta] = min(r_mean);
min_theta = min_theta - 1;
theta = linspace(0,2*pi,361)';
theta = theta(1:360);
%polar(theta,r_mean)
robot.stopLaser()

y = sin(deg2rad(min_theta))*min_r;
x = cos(deg2rad(min_theta))*min_r;
end


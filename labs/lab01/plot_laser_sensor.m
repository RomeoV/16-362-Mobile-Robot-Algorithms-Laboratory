close all
robot.startLaser()
pause(3)
tic()
r_matrix = zeros(360,40);
index = 1;
while toc() < 4
    r_matrix(:,index) = circshift(robot.laser.LatestMessage.Ranges,-5);
    index=index+1;
    pause(.1)
end
r_matrix(r_matrix>2) = 0;
r_matrix(isoutlier(r_matrix,2)) = nan;
polar(linspace(0,2*pi,360)',mean(r_matrix,2,'omitnan'))
robot.stopLaser()

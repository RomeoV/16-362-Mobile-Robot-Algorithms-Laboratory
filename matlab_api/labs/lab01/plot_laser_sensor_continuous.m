clear;
close all;
if ~exist('robot')
    robot = raspbot('move_x_seconds')
end

robot.startLaser()
pause(3)
tic()
r_matrix = zeros(360,5);
figure();
polar(linspace(0,2*pi,360),ones(1,360));
while toc() < 40
    for i = 1:5
        r_matrix = circshift(r_matrix,-1,2);
        r_matrix(:,40) = circshift(robot.laser.LatestMessage.Ranges,-5);
        pause(1/20);
    end
    r_matrix(r_matrix>2) = 0;
    %r_matrix(isoutlier(r_matrix,2)) = nan;
    polar(linspace(0,2*pi,360)',mean(r_matrix,2,'omitnan'));
    drawnow;
end
robot.stopLaser()

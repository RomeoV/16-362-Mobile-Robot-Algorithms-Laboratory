clear;
close all;
if ~exist('robot')
    robot = raspbot('move_x_seconds')
end
figure

robot.startLaser()
pause(2);
tic()
while toc() < 7
    robot.sendVelocity(.25,.25);
    r = circshift(robot.laser.LatestMessage.Ranges',5);
    r(r>1) = 0;
    polar(linspace(0,2*pi,360),r);
    drawnow();
    pause(.1);
end
robot.stop();

tic()
while toc() < 14
    robot.sendVelocity(-.25,-.25);
    r = circshift(robot.laser.LatestMessage.Ranges',5);
    r(r>1) = 0;
    polar(linspace(0,2*pi,360),r);
    drawnow();
    pause(.1);
end

robot.stopLaser();
robot.stop();
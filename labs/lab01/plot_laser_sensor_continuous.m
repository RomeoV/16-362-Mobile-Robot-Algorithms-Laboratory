sys.robot.startLaser()
pause(3)
tic()
r_matrix = zeros(360,5);
figure();
polar(linspace(0,2*pi,360),ones(1,360));
while toc() < 40
    r_values = circshift(robot.laser.LatestMessage.Ranges,robotModel.laserOffset);
    r_values(r_values>1.5) = 0;
    r_values(r_values<0.08)= 0;
    polar(linspace(0,2*pi,360)',r_values);
    drawnow;
    pause(0.05);
end
robot.stopLaser()

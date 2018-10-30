robot = sys.robot;

robot.startLaser()
pause(3)
tic()
r_data = zeros(360,1);
figure(2);
polar(linspace(0,2*pi,360),ones(1,360));
while toc() < 100
    r_data = circshift(robot.laser.LatestMessage.Ranges,robotModel.laserOffset);
    r_data(r_data>0.8) = 0;
    r_data(r_data<0.08) = 0;
    polar(linspace(0,2*pi,360)',r_data);
    drawnow;
    pause(0.01);
end
robot.stopLaser()

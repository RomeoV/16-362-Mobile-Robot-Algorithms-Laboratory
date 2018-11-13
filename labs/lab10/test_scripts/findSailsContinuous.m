robot = sys.robot;

robot.startLaser()
pause(3)
tic()
while toc() < 100
    range_image = rangeImage(robot.laser.LatestMessage.Ranges());
    [sails, walls] = range_image.findSailsAndWalls();
    range_image.plotXvsY();
    range_image.plotSails(sails);
    range_image.plotWalls(walls);
    drawnow;
    pause(0.02);
end
robot.stopLaser()

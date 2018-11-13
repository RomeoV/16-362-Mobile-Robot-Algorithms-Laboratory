clc;
close all;

robot = raspbot('good boi')

%4 feet = 1.22 meters

p1 = [0.2 ; 0];
p2 = [ 1.22 ; 0];
p3 = [0; 0.2];
p4 = [0 ; 1.22];

lines_p1 = [p1 p3];
lines_p2 = [p2 p4];

plot([0 1.22],[0 0],'b')
hold on
plot([0 0],[0 1.22],'b')

gain = 0.3;
errThresh = 0.01;
gradThresh = 0.0005;

robotPose = pose(12*0.0254,12*0.0254,pi()/2.0);

obj = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)
drive = robotKeypressDriver(gcf);

robot.startLaser();
pause(2);

tic();
firstIter = true;
drive.drive(robot,1)

lidar = scatter(0,0,'r');

robo = scatter(robotPose.x,robotPose.y,'g')

xlim([-1 3])
ylim([-1 3])

while toc() < 60 
  
    drive.drive(robot,1)
    
    r_values = circshift(robot.laser.LatestMessage.Ranges,robotModel.laserOffset);
    
    th = linspace(1,360,360)';
    
    goodones = r_values>0.08 & r_values<1.5;
    r_values = r_values(goodones);
    th = th(goodones);
    
    every_10 = mod(th,10)==0;
    r_values = r_values(every_10);
    th = th(every_10);
    theta = deg2rad(th);
    
    x = r_values.*cos(theta);
    y = r_values.*sin(theta);
    
    
    if ~isempty(theta)
        modelPts = [x';y';ones(1,size(x',2))];
        [success, outPose] = obj.refinePose(robotPose,modelPts,1000);
        
        worldPts = robotPose.bToA()*modelPts;
%         no_corner = (worldPts(1,:).^2 + worldPts(2,:).^2)>0.02;
%         worldPts = worldPts(:,no_corner);
        lidar.XData = worldPts(1,:);
        lidar.YData = worldPts(2,:);
        if success
            robotPose = outPose;
        else
            %disp("shit")
        end
        %disp(robotPose.getPoseVec)
        robo.XData = robotPose.x;
        robo.YData = robotPose.y;
    end
    
    pause(0.05)
end

robot.stop();
robot.stopLaser()
pause(2);
robot.shutdown();
%4 feet = 1.22 meters

p1 = [0.1 ; 0];
p2 = [ 1.22 ; 0];
p3 = [ 0 ; 0.1];
p4 = [ 0 ; 1.22];

lines_p1 = [p1 p3];
lines_p2 = [p2 p4];

gain = 0.3;
errThresh = 0.01;
gradThresh = 0.0005;

robotPose = pose(24*0.0254,12*0.0254,pi()/2.0);

obj = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)
drive = robotKeypressDriver(gcf);

robot.startLaser();
pause(2);

tic();
firstIter = true;
drive.drive(robot,1)

bodyPts = robotModel.bodyGraph();

r_values = circshift(robot.laser.LatestMessage.Ranges,robotModel.laserOffset);
    
    th = linspace(1,360,360)';
    
    goodones = r_values>0.08 & r_values<1;
    r_values = r_values(goodones);
    th = th(goodones);
    
    every_10 = mod(th,2)==0;
    r_values = r_values(every_10);
    th = th(every_10);
    theta = deg2rad(th);
    
    x = r_values.*cos(theta);
    y = r_values.*sin(theta);
    
    
    if ~isempty(theta)
        modelPts = [x';y';ones(1,size(x',2))];
        [success, outPose] = obj.refinePose(robotPose,modelPts,100);
        
        
        world_lider_xy = robotPose.bToA()*modelPts;
        modelPts(1:2,:) = world_lider_xy(1:2,:);
        [success, outPose] = obj.refinePose(robotPose,modelPts,100);
        if success
            robotPose = outPose;
        else
            disp("failure");
        end
    else
        disp("empty");
    end
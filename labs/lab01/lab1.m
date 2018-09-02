function lab1
clc
clear;
close all;
addpath(genpath('matlab_api'));
disp("setup robot");
%if you don't run it through the simulator it wont work immediately.
robot = raspbot('notsim')

disp('setup done');

% Move the robot forward a certain distance
disp('move 12 in forward');

robot.stop();
dist = 0;
target_dist = 0.3; %12 cm
prev_encoderX = robot.encoders.LatestMessage.Vector.X;
move_xcm_straight(robot,target_dist,dist,prev_encoderX,0.07);

robot.stop();
pause(1);
disp('move 12 in backward');

dist = 0;
target_dist = 0.3; %12 cm
prev_encoderX = robot.encoders.LatestMessage.Vector.X;
move_xcm_straight(robot,target_dist,dist,prev_encoderX,-0.07);


%end
disp('ended');
robot.stop();
robot.shutdown();
end

function move_xcm_straight(robot,target_dist,dist,prev_encoderX,max_speed)
    %wheel left right bias about left = 0.965*right

    while(abs(dist)<target_dist)
        
    if((target_dist-abs(dist))>0.2*target_dist)
        robot.sendVelocity(max_speed, 0.965*max_speed)
    else
        robot.sendVelocity(max_speed/2, 0.965*max_speed/2)
    end
    

    if(~isnan(robot.encoders.LatestMessage.Vector.X))
        dist= robot.encoders.LatestMessage.Vector.X-prev_encoderX
    end
    pause(.05)
    end
end
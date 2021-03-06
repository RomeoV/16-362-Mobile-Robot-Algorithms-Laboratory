clear; close all;
if exist('robot') ~= 1
    robot = raspbot('hamilton'); 
end
trajectory = robotTrajectory();
log_data = logger();
log_data.logging = true;
real_time_plotting = false;
pallet_d = -0.3;
if(real_time_plotting)
    subplot(1,3,1);
    hold on;
    plot(log_data.x_est_data, log_data.y_est_data);
    plot(log_data.x_data, log_data.y_data);
    xlabel('x');
    ylabel('y');
    %legend('estimation','target');
    subplot(1,3,3)
    %       legend('x error', 'y error', 'theta error')
    %legend('ref', 'est');
end

robot.forksDown();
pause(1);

est_robot = estRobot(robot.encoders.LatestMessage.Vector.X,...
                     robot.encoders.LatestMessage.Vector.Y);
ref_robot = refRobot(trajectory);

current_time = 2;

% Start laser

robot.startLaser()
pause(2.5);

find_closest_sail;

disp(sail(:))

robot_frame = pose([0 0 0]);

goal_pose = pose(sail(:,1)'); %local

final_pose = goal_pose;

fork_pose = pose(-0.20,0,0);
    
goal_pose_in_rf = pose(pose.matToPoseVec(robot_frame.bToA()*goal_pose.bToA()));

goal_pose = pose(pose.matToPoseVec(goal_pose_in_rf.bToA()*fork_pose.bToA()));

trajectory.generateTraj(goal_pose.x(),...
                        goal_pose.y(),...
                        goal_pose.th(),...
                        1,0.2);

execute_trajectory

[cx, cy] = find_close_sails(robot);
disp(cx + " : " + cy);

th_goal = atan2(cy,cx);
th_goal = deg2rad(th_goal);
disp("th_goal: " + th_goal);

th_i = est_robot.theta_est;
th_robo = 0;
tstamp = double(robot.encoders.LatestMessage.Header.Stamp.Sec)+double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;

robot.stop();

if (abs(atan2(cy,cx)))>2
    while rad2deg(abs(th_robo-th_goal)) > 2
          error_th = th_goal-th_robo;

          robot.sendVelocity(0.01*sign(error_th),-0.01*sign(error_th));

          encoder_l = robot.encoders.LatestMessage.Vector.X;
          encoder_r = robot.encoders.LatestMessage.Vector.Y;
          th_robo = th_i-est_robot.theta_est;
          last_tstamp = tstamp;
          tstamp = double(robot.encoders.LatestMessage.Header.Stamp.Sec)+double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
          d_tstamp = tstamp-last_tstamp;
          %disp("Matlab dt: " + dt + " - Robot dt: " + d_tstamp)
          %tstamp = current_time;
          est_robot.updateEstimation(d_tstamp,encoder_l,encoder_r);
          pause(0.05)
    end
    [cx, cy] = find_close_sails(robot);

    th_goal = atan2(cy,cx);
    th_goal = deg2rad(th_goal);
    disp("final offset: " + th_goal)
else
    disp("We Good");
end
robot.stop();

pause(0.05);

tic()
while toc()<1
robot.sendVelocity(0.1,0.1);
encoder_l = robot.encoders.LatestMessage.Vector.X;
encoder_r = robot.encoders.LatestMessage.Vector.Y;
th_robo = th_i-est_robot.theta_est;
last_tstamp = tstamp;
tstamp = double(robot.encoders.LatestMessage.Header.Stamp.Sec)+double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
d_tstamp = tstamp-last_tstamp;
%disp("Matlab dt: " + dt + " - Robot dt: " + d_tstamp)
%tstamp = current_time;
est_robot.updateEstimation(d_tstamp,encoder_l,encoder_r);
pause(0.05);
end
robot.forksUp();
pause(1);

% goal_pose = pose(0,0,0);
% goal_pose_in_rf = pose(pose.matToPoseVec(robot_frame.bToA()*goal_pose.bToA()));
% 
% trajectory.generateTraj(goal_pose.x(),...
%                         goal_pose.y(),...
%                         goal_pose.th(),...
%                         1,0.2);
% 
% execute_trajectory

    
find_closest_sail
disp("Turning " + rad2deg(sail(3)) + " degrees!");
rotate_robot(robot,sail(3));
robot.forksDown();
pause(2);

robot.sendVelocity(0.1,0.1);
%%

pause(3);
robot.forksUp();
pause(1);

    
robot.stop()
robot.stopLaser();
robot.shutdown();

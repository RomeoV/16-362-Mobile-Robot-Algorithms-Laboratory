clear; close all;
if exist('robot') ~= 1
    robot = raspbot('hamilton'); 
end
trajectory = robotTrajectory();
log_data = logger();
log_data.logging = true;
real_time_plotting = true;
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

est_robot = estRobot(robot.encoders.LatestMessage.Vector.X,...
                     robot.encoders.LatestMessage.Vector.Y);
ref_robot = refRobot(trajectory);

current_time = 0;

goals = ...
    [0, 0.3048,-0.6096,-0.3048;
     0, 0.3048,-0.6096,0.3048 ;
     0, 0,     -pi/2,  pi/2   ];
robot_frame = pose(goals(:,1)');
for i = 2:size(goals,2)
    goal_pose = pose(goals(:,i)'); %local
    goSomewhere
    pause()
end


% robot_frame = est_robot.getPose();
% goal_pose_1 = pose(3*0.3048, 3*0.3048, 0);
% % translate goal pose to robot frame
% goal_pose_in_rf = pose(pose.matToPoseVec(robot_frame.bToA()*goal_pose_1.bToA()));
% trajectory.generateTraj(goal_pose_in_rf.x(),...
%                         goal_pose_in_rf.y(),...
%                         goal_pose_in_rf.th(),...
%                         1,0.1);
% execute_trajectory
% 
% robot_frame = est_robot.getPose();
% goal_pose_1 = pose(3*0.6, 0, 0);
% % translate goal pose to robot frame
% goal_pose_in_rf = pose(pose.matToPoseVec(robot_frame.aToB()*goal_pose_1.bToA()));
% trajectory.generateTraj(goal_pose_in_rf.x(),...
%                         goal_pose_in_rf.y(),...
%                         goal_pose_in_rf.th(),...
%                         1,0.1);
% execute_trajectory


robot.shutdown();

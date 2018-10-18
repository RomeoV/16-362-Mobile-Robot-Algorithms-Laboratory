clear; close all;
robot = raspbot('hamilton');
trajectory = robotTrajectory();
log_data = logger();
log_data.logging = true;
real_time_plotting = true;
if(real_time_plotting)
    figure;
    hold on;
    plot(log_data.x_est_data, log_data.y_est_data);
    plot(log_data.x_data, log_data.y_data);
    xlabel('x');
    ylabel('y');
    legend('ref', 'est');
end

est_robot = estRobot(robot.encoders.LatestMessage.Vector.X,...
                     robot.encoders.LatestMessage.Vector.Y);
ref_robot = refRobot(trajectory);

current_time = 0;

robot_frame = est_robot.getPose();
goal_pose_1 = pose(0.3048, 0.3048, 0);
% translate goal pose to robot frame
goal_pose_in_rf = pose(pose.matToPoseVec(robot_frame.bToA()*goal_pose_1.bToA()));
trajectory.generateTraj(goal_pose_in_rf.x(),...
                        goal_pose_in_rf.y(),...
                        goal_pose_in_rf.th(),...
                        1,0.1);
execute_trajectory

robot_frame = est_robot.getPose();
goal_pose_1 = pose(0.6, 0, 0);
% translate goal pose to robot frame
goal_pose_in_rf = pose(pose.matToPoseVec(robot_frame.aToB()*goal_pose_1.bToA()));
trajectory.generateTraj(goal_pose_in_rf.x(),...
                        goal_pose_in_rf.y(),...
                        goal_pose_in_rf.th(),...
                        1,0.2);
execute_trajectory

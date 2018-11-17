function [x,y] = lidar_in_world(robot_pose_est,lidar_data)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
plot_real_time = false;
indices = lidar_data > 0.8 & lidar_data < 1.5;
th = linspace(0,2*pi,360)';
plt_x = cos(th).*lidar_data;
plt_y = sin(th).*lidar_data;
walls_in_world_frame_x = zeros(360,1);
walls_in_world_frame_y = zeros(360,1);
for i = indices
    p = pose(pose.matToPoseVec(...
        robot_pose_est.bToA()*pose(plt_x(i),plt_y(i),0).bToA()));
    walls_in_world_frame_x(i) = p.x();
    walls_in_world_frame_y(i) = p.y();
end
bad_indices = walls_in_world_frame_x < 0.30 & -walls_in_world_frame_y < 0.30 | ...
    walls_in_world_frame_x > 0.70 | -walls_in_world_frame_y > 0.70;
walls_in_world_frame_x(bad_indices) = robot_pose_est.x();
walls_in_world_frame_y(bad_indices) = robot_pose_est.y();

if plot_real_time
    close all
    figure
    plot(-plt_y,plt_x); hold on;
    plot(-walls_in_world_frame_y,walls_in_world_frame_x);
    daspect([1,1,1]);
end
x = walls_in_world_frame_x;
y = walls_in_world_frame_y;
end


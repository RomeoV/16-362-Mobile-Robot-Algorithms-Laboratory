function error = pose_estimate_error(pose_est_vec, lidar_data, indices_x_axis, indices_y_axis)
%ESTIMATE_POSITION Summary of this function goes here
%   Detailed explanation goes here
pose_est = pose(pose_est_vec);
range_image = rangeImage(lidar_data);
[x,y] = range_image.getXYinWorldCoords(pose_est);

[error_x_axis,~]  = closestPointOnLineSegment([x(indices_x_axis),y(indices_x_axis)]',[0.;0],[1.25;0]);
[error_y_axis,~] = closestPointOnLineSegment([x(indices_y_axis),y(indices_y_axis)]',[0;0],[0;1.25]);
error = [error_x_axis';error_y_axis'];
end


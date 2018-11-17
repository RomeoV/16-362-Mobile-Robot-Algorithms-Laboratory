opts = optimoptions(@lsqnonlin);
opts.Display = 'iter';

robot_pose_vec = [.3;.3;0];
range_image = rangeImage(lidar_data);
[x,y] = range_image.getXYinWorldCoords(pose(robot_pose_vec));

dist = 0.25;
indices_x_axis = y < dist & x < 1-dist & x > dist;
indices_y_axis = x < dist & y < 1-dist & y > dist;

obj = @(pose_vec) pose_estimate_error(pose_vec, lidar_data, indices_x_axis, indices_y_axis);

lsqnonlin(obj,robot_pose_vec,[],[],opts)
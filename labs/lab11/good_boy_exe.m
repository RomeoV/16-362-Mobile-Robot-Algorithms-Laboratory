clc;
sys = mrplSystem();
sys.real_time_plotting=false;
close all;

opts = optimoptions(@lsqnonlin);
opts.Display = 'none';
opts.UseParallel = false;
opts.OptimalityTolerance = 1e-3;
opts.MaxIterations = 100;

figure(1)
scatter([0],[0])
daspect([1,1,1])
xlim([-.5,1.5])
ylim([-.5,1.5])

robot_pose_vec = [.3;.3;0];
drive = robotKeypressDriver(gcf);

first_iter = true;
tic()
while toc() < 60
    if first_iter
        tic()
        first_iter = false;
    end
    
    drive.drive(sys.robot,1);
    
    lidar_data = sys.robot.laser.LatestMessage.Ranges;
    range_image = rangeImage(lidar_data);
    [x,y] = range_image.getXYinWorldCoords(pose(robot_pose_vec));

    dist = 0.15;
    indices_x_axis = y < 0.05 & x < 1-dist & x > dist;
    indices_y_axis = x < 0.05 & y < 1-dist & y > dist;

    obj = @(pose_vec) pose_estimate_error(pose_vec, lidar_data, indices_x_axis, indices_y_axis);

    robot_pose_vec = lsqnonlin(obj,robot_pose_vec,[],[],opts);
    
    indices_plt = indices_x_axis | indices_y_axis;
    hold off; scatter(x(indices_plt),y(indices_plt),'.r');
    hold on; plot([0;0],[0.1;0.9],'g')
    plot([0.1;0.9],[0;0],'g'); hold off;
    drawnow();
    pause(0.005);
end
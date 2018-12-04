classdef StateEstimator < handle
properties
  updateInterval = 1;
  internalTimerID
  odometry_estimator
  real_time_plotting
  delta_data
end

methods(Static=true)
  function opts = getOptimOpts()
    opts = optimoptions(@lsqnonlin);
    opts.Display = 'none';
    %opts.Display = 'final-detailed';
    opts.UseParallel = false;
    opts.OptimalityTolerance = 1e-6;
    opts.FunctionTolerance = 1e-4;
    opts.MaxIterations = 100;
    opts.FunValCheck = 'on';
    %opts.Algorithm = 'levenberg-marquardt';
  end
end

methods
  function obj = StateEstimator(initial_pose,enc_vals,real_time_plotting)
    %%STATEESTIMATOR Uses odometry and lidar for pose estimation
    % initial_pose -> pose object of where the robot is initially
    % enc_vals -> size 2 vector with left and right wheel initial encoder values
    % real_time_plotting -> Plot lidar graphs on the fly

    obj.odometry_estimator = OdometryEstimator(enc_vals(1),enc_vals(2));
    obj.odometry_estimator.setPose(initial_pose);
    obj.internalTimerID = tic();
    obj.real_time_plotting = real_time_plotting;
    
    obj.delta_data = [0;0];
  end

  function pose = getPose(obj)
    pose = obj.odometry_estimator.getPose();
  end

  function updateOdometry(obj, timestamp, enc_vals)
    obj.odometry_estimator.updateEstimation(timestamp, enc_vals(1), enc_vals(2));
  end

  function spin(obj,robot)
    if toc(obj.internalTimerID) > obj.updateInterval
      obj.internalTimerID = tic();

      robot_pose_est = obj.odometry_estimator.getPose();
      lidar_data = robot.laser.LatestMessage.Ranges;
      range_image = rangeImage(lidar_data);
      [x,y] = range_image.getXYinWorldCoords(robot_pose_est);

      dist = 0.25;
      indices_x_axis = y > -0.10 & y < 0.10 & x < 1-dist & x > dist;% & mod(1:length(x),4)' == 0;
      indices_y_axis = x > -0.10 & x < 0.10 & y < 1-dist & y > dist;% & mod(1:length(y),4)' == 0;
      %lidar_data(~(indices_x_axis | indices_y_axis)) = 0;
      %range_image = range_image.getXYInWorldCoords(robot_pose_est);
      %lidar_data = lidar_data(indices_x_axis | indices_y_axis);
      %range_image = rangeImage(lidar_data);
        
      disp("x: " + num2str(sum(indices_x_axis)) + " - y: " + num2str(sum(indices_y_axis)));
      if sum(indices_x_axis) >= 5 && sum(indices_y_axis) >= 5
          objective = @(pose_vec) pose_estimate_error(pose_vec, lidar_data, indices_x_axis, indices_y_axis);

          optim_timer = tic();
          new_robot_pose_vec = lsqnonlin(objective,robot_pose_est.getPoseVec(),[],[],StateEstimator.getOptimOpts());
          disp("Optimtime: " + num2str(toc(optim_timer)));

          delta_pose = new_robot_pose_vec-robot_pose_est.getPoseVec();
          delta_pose(3) = delta_pose(3)*0.8;
          obj.delta_data = [obj.delta_data [norm(delta_pose(1:2));delta_pose(3)]];
          fusion_pose_vec = robot_pose_est.getPoseVec() + 0.25*delta_pose;
          fusion_pose_vec(3) = wrapToPi(fusion_pose_vec(3));      
          obj.odometry_estimator.setPose(pose(fusion_pose_vec));
      
      
      else
        disp('Skipping lidar correction')
      end
      if obj.real_time_plotting
        indices_plt = indices_x_axis | indices_y_axis;
        figure(12)
        hold off; scatter(x(indices_plt),y(indices_plt),'.r');
        hold on; plot([0;0],[0;1],'g')
        plot([0;1],[0;0],'g'); 
        robot_pose_est = obj.odometry_estimator.getPose();
        quiver(robot_pose_est.x(), robot_pose_est.y(),...
           -cos(robot_pose_est.th()),-sin(robot_pose_est.th()),0.05,'filled','-.xr','LineWidth',2); hold off;
       figure(13)
       plot(obj.delta_data(1,:)); hold on
       plot(obj.delta_data(2,:)); hold off
       title('Lidar correction (x/y, theta)')
       drawnow();
     end

    end
  end
end
end
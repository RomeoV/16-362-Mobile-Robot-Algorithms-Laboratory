classdef StateEstimator < handle
properties
  updateInterval = 1
  internalTimerID
  odometry_estimator
  real_time_plotting
end

methods(Static=true)
  function opts = getOptimOpts()
    opts = optimoptions(@lsqnonlin);
    opts.Display = 'none';
    opts.UseParallel = false;
    opts.OptimalityTolerance = 1e-3;
    opts.MaxIterations = 100;
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

      dist = 0.15;
      indices_x_axis = y < 0.05 & x < 1-dist & x > dist;
      indices_y_axis = x < 0.05 & y < 1-dist & y > dist;

      objective = @(pose_vec) pose_estimate_error(pose_vec, lidar_data, indices_x_axis, indices_y_axis);

      new_robot_pose_vec = lsqnonlin(objective,robot_pose_est.getPoseVec(),[],[],StateEstimator.getOptimOpts());
      
      fusion_pose_vec = robot_pose_est.getPoseVec() + 0.25*(new_robot_pose_vec-robot_pose_est.getPoseVec());
      fusion_pose_vec(3) = wrapToPi(fusion_pose_vec(3));      
      obj.odometry_estimator.setPose(pose(fusion_pose_vec));
      
      if obj.real_time_plotting
        indices_plt = indices_x_axis | indices_y_axis;
        figure(12)
        hold off; scatter(x(indices_plt),y(indices_plt),'.r');
        hold on; plot([0;0],[0;1],'g')
        plot([0;1],[0;0],'g'); hold off;
        drawnow();
        figure(11)
      end
    end
  end
end
end
tic();
firstIter = true;
while toc() < max(trajectory.t_eval+2*robotModel.tdelay)
  if firstIter
      tic();
      firstIter = false;
      ref_robot.trajectory_starting_time = toc();
      tstamp = double(robot.encoders.LatestMessage.Header.Stamp.Sec)+double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
      pause(0.001);
  end
  last_time = current_time;
  current_time = toc();
  dt = current_time - last_time;
  %disp(dt + " : " + est_robot.dtime)
  encoder_l = robot.encoders.LatestMessage.Vector.X;
  encoder_r = robot.encoders.LatestMessage.Vector.Y;

  last_tstamp = tstamp;
  tstamp = double(robot.encoders.LatestMessage.Header.Stamp.Sec)+double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
  d_tstamp = tstamp-last_tstamp;
  disp("Matlab dt: " + dt + " - Robot dt: " + d_tstamp)
  %tstamp = current_time;
  
  est_robot.updateEstimation(d_tstamp,encoder_l,encoder_r);
  
  delayed_time = max(0,current_time-robotModel.tdelay);
  %pose_ref_in_rf = ref_robot.getReferencePoseAtTime(current_time-robotModel.tdelay);
  pose_ref_in_rf = pose(trajectory.x_at_time(delayed_time),...
      trajectory.y_at_time(delayed_time),...
      trajectory.theta_at_time(delayed_time));
  
  pose_ref_in_wf = pose(pose.matToPoseVec(robot_frame.bToA()*pose_ref_in_rf.bToA()));
  
  pose_est = est_robot.getPose();
  
  vl_ffd = trajectory.vl_at_time(delayed_time);
  
  vr_ffd = trajectory.vr_at_time(delayed_time);
  
  [vl_control, vr_control] = controller.getControlInput(pose_ref_in_wf, pose_est,...
      vl_ffd, vr_ffd,...
      trajectory.V_at_time(current_time-robotModel.tdelay),log_data);
  
    log_data.log_data(pose_est.x(), pose_est.y(), pose_est.th(), encoder_l, encoder_r, current_time, pose_ref_in_wf.x(), pose_ref_in_wf.y(), pose_ref_in_wf.th());
  
      % Send control to plant
    if isnan(vl_control) || isnan(vr_control)
        robot.sendVelocity(0, 0);
    else
        robot.sendVelocity(vl_control, vr_control); 
    end
  
  plot_graphs
  drawnow();
  pause(0.005);
end
robot_frame = pose(pose.matToPoseVec(robot_frame.bToA()*goal_pose.bToA()));

tic();
firstIter = true;
while toc() < max(trajectory.t_eval+robotModel.tdelay)
  if firstIter
      tic();
      firstIter = false;
      ref_robot.trajectory_starting_time = toc();
  end
  last_time = current_time;
  current_time = toc();
  dt = current_time - last_time;
  encoder_l = robot.encoders.LatestMessage.Vector.X;
  encoder_r = robot.encoders.LatestMessage.Vector.Y;

  est_robot.updateEstimation(current_time,encoder_l,encoder_r);
  
  pose_ref_in_rf = ref_robot.getReferencePoseAtTime(current_time-robotModel.tdelay);
  pose_ref_in_wf = pose(pose.matToPoseVec(robot_frame.bToA()*pose_ref_in_rf.bToA()));
  pose_est = est_robot.getPose();
  vl_ffd = trajectory.vl_at_time(current_time-robotModel.tdelay);
  vr_ffd = trajectory.vr_at_time(current_time-robotModel.tdelay);
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

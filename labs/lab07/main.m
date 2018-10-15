clear; close all;
robot = raspbot('sim');
trajectory = robotTrajectory();

est_robot = estRobot(robot.encoders.LatestMessage.Vector.X,...
                     robot.encoders.LatestMessage.Vector.Y);
ref_robot = refRobot(trajectory);

firstIter = true;
current_time = 0;
trajectory.generateTraj(0.3048,0.3048,0,1,0.2);
tic();
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
  
  pose_ref = ref_robot.getReferencePoseAtTime(current_time);
  pose_est = est_robot.getPose();
  vl_ffd = trajectory.vl_at_time(current_time);
  vr_ffd = trajectory.vr_at_time(current_time);
  [vl_control, vr_control] = controller.getControlInput(pose_ref, pose_est,...
      vl_ffd, vr_ffd,...
      trajectory.V_at_time(current_time));
  
      % Send control to plant
    if isnan(vl_control) || isnan(vr_control)
        robot.sendVelocity(0, 0);
    else
        robot.sendVelocity(vl_control, vr_control); 
    end
  
  pause(0.005);
end

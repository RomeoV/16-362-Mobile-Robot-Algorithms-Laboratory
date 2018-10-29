classdef refRobot < handle
properties
  trajectory
  trajectory_starting_pose
  trajectory_starting_time
end

methods
  function obj = refRobot(traj)
    obj.trajectory = traj;
    obj.trajectory_starting_pose = pose(0,0,0);
    obj.trajectory_starting_time = 0;
  end

  function setNewTrajectory(obj,traj,pose,time)
    obj.trajectory = traj;
    obj.trajectory_starting_pose = pose;
    obj.trajectory_starting_time = time;
  end

  function p = getReferencePoseAtTime(obj, time)
    %getreferenceposeattime Returns Pose in world frame
    local_time = time-obj.trajectory_starting_time;
    x = obj.trajectory.x_at_time(local_time);
    y = obj.trajectory.y_at_time(local_time);
    t = obj.trajectory.theta_at_time(local_time);

    p_local = pose(x,y,t);
    p = pose(pose.matToPoseVec(...
      obj.trajectory_starting_pose.bToA()*...
      p_local.bToA()));
  end
end

end

classdef controller < handle
  properties
    pid
  end

  methods(Access = public)
    function obj = controller(pid_)
      pid = pid_;
    end
  end

methods(Static = true)
  function [vl, vr] = getControlInput(pose_ref, pose_est, vl_ffd, vr_ffd, V)
    tau = 1.2;
    k_x = 1/tau;
    k_y = 2/(tau^2*abs(V));
    k_theta = 1/tau;


    error_x = pose_ref.x() - pose_est.x();
    error_y = pose_ref.y() - pose_est.y();
    error_th = pose_ref.th() - pose_est.th();

    error_rel = [cos(pose_est.th()) sin(pose_est.th()); -sin(pose_est.th()) cos(pose_est.th())]*[error_x;error_y];
    if error_rel(2) < 0.005; k_y = 0; end

    error_V = k_x*error_rel(1);
    error_omega = k_y*error_rel(2)+k_theta*error_th;
    [vl_fb, vr_fb] = robotModel.VwTovlvr(error_V, error_omega);

    pid = 1;
    [vl, vr] = robotModel.limitWheelVelocities([vl_ffd+pid*vl_fb; vr_ffd+pid*vr_fb]);
  end
end


end

classdef controller < handle

methods(Static = true)
  function [vl, vr] = getControlInput(pose_ref, pose_est, vl_ffd, vr_ffd, V, log_data)
    tau = 1;
    k_x = 1/tau;
    if (V < .005)
        k_y = 0;
        k_theta = 0
    else
        k_y = 2/(tau^2*abs(V));
        k_theta = 1/tau;
    end
   


    error_x = pose_ref.x() - pose_est.x();
    error_y = pose_ref.y() - pose_est.y();
    error_th = pose_ref.th() - pose_est.th();
    error_th = atan2(sin(error_th),cos(error_th));
    
    log_data.log_errors(error_x, error_y, error_th);

    error_rel = [cos(pose_est.th()) sin(pose_est.th()); -sin(pose_est.th()) cos(pose_est.th())]*[error_x;error_y];
    %if abs(error_rel(2)) < 0.002; k_y = 0; end
    

    error_V = k_x*error_rel(1);
    error_omega = k_y*error_rel(2)+k_theta*error_th;
%     if V<0.005
%         error_omega = sign(error_omega)*(min(abs(error_omega),1));
%     end
    
    [vl_fb, vr_fb] = robotModel.VwTovlvr(error_V, error_omega);

    pid = 1;
    [vl, vr] = robotModel.limitWheelVelocities([vl_ffd+pid*vl_fb; vr_ffd+pid*vr_fb]);
  end
end


end

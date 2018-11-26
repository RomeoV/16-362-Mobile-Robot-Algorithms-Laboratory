classdef OdometryEstimator < handle
properties
  encoder_l_prev
  encoder_r_prev
  x_est = nan
  y_est = nan
  theta_est = nan
  last_odometry_timestamp = nan
end

methods
  function obj = OdometryEstimator(l_initial, r_initial)
    obj.encoder_l_prev = l_initial;
    obj.encoder_r_prev = r_initial;
  end

  function setPose(obj, pose)
    obj.x_est = pose.x();
    obj.y_est = pose.y();
    obj.theta_est = pose.th();
  end

  function p = getPose(obj)
    p = pose(obj.x_est,obj.y_est,obj.theta_est);
  end

  function updateEstimation(obj,timestamp, encoder_l, encoder_r)
    if isnan(obj.last_odometry_timestamp)
      obj.last_odometry_timestamp = timestamp;
    else
      dt = timestamp - obj.last_odometry_timestamp;
      obj.last_odometry_timestamp = timestamp;

      if dt <= 0
          vl_est = 0;
          vr_est = 0;
      elseif dt > 0
          vl_est = (encoder_l-obj.encoder_l_prev)/dt;
          vr_est = (encoder_r-obj.encoder_r_prev)/dt;
      end
      
      obj.encoder_l_prev = encoder_l; obj.encoder_r_prev = encoder_r;

      [V_est, omega_est] = robotModel.vlvrToVw(vl_est, vr_est);
      obj.x_est = obj.x_est + cos(obj.theta_est)*V_est*dt/2;
      obj.y_est = obj.y_est + sin(obj.theta_est)*V_est*dt/2;
      obj.theta_est = wrapToPi(obj.theta_est + omega_est*dt);
      obj.x_est = obj.x_est + cos(obj.theta_est)*V_est*dt/2;
      obj.y_est = obj.y_est + sin(obj.theta_est)*V_est*dt/2;
    end
  end

end
end

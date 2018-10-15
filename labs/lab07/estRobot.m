classdef estRobot < handle
properties
  timestamp_prev
  encoder_l_prev
  encoder_r_prev
  x_est
  y_est
  theta_est
end

methods
  function obj = estRobot(l_initial, r_initial)
    obj.encoder_l_prev = l_initial;
    obj.encoder_r_prev = r_initial;
    obj.timestamp_prev = 0;
    obj.x_est = 0;
    obj.y_est = 0;
    obj.theta_est = 0;
  end

  function updateEstimation(obj,timestamp, encoder_l, encoder_r)
    dt = timestamp - obj.timestamp_prev; obj.timestamp_prev = timestamp;
    vl_est = (encoder_l-obj.encoder_l_prev)/dt;
    vr_est = (encoder_r-obj.encoder_r_prev)/dt;
    obj.encoder_l_prev = encoder_l; obj.encoder_r_prev = encoder_r;

    [V_est, omega_est] = robotModel.vlvrToVw(vl_est, vr_est);
    obj.x_est = obj.x_est + cos(obj.theta_est)*V_est*dt/2;
    obj.y_est = obj.y_est + sin(obj.theta_est)*V_est*dt/2;
    obj.theta_est = obj.theta_est + omega_est*dt;
    obj.x_est = obj.x_est + cos(obj.theta_est)*V_est*dt/2;
    obj.y_est = obj.y_est + sin(obj.theta_est)*V_est*dt/2;
  end

  function p = getPose(obj)
    p = pose(obj.x_est,obj.y_est,obj.theta_est);
  end

end
end

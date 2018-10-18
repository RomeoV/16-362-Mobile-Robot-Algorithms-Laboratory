classdef logger < handle
  properties
    x_est_data
    y_est_data
    theta_est_data
    wl_data
    wr_data
    t_data
    error_x_data
    error_y_data
    error_theta_data
    x_data
    y_data
    logging
  end

  methods(Access = public)
    function log_data(obj, x_est, y_est, theta_est, encoder_l, encoder_r, time, x_ref, y_ref)
      if(obj.logging)    
        obj.x_est_data = [obj.x_est_data x_est];
        obj.y_est_data = [obj.y_est_data y_est];
        obj.theta_est_data = [obj.theta_est_data theta_est];
        obj.wl_data = [obj.wl_data encoder_l];
        obj.wr_data = [obj.wr_data encoder_r];
        obj.t_data = [obj.t_data time];
        obj.x_data = [obj.x_data x_ref];
        obj.y_data = [obj.y_data y_ref];
      end
    end
    function log_errors(obj,err_x, err_y, err_theta)
      if(obj.logging)
        obj.error_x_data = [obj.error_x_data err_x];
        obj.error_y_data = [obj.error_y_data err_y];
        obj.error_theta_data = [obj.error_theta_data err_theta];
      end
    end
  end
end

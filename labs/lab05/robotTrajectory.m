classdef robotTrajectory < handle
    %ROBOTTRAJECTORY Generates a trajectory with pose and velocity 
    %   Sampled data (_eval) as well as interpolated data (_at_time(t)) is
    %   provided
    
    properties
        % The _eval variables are sampled data
        % Use the _at_time(t) methods for interpolated data
        t_eval
        V_eval
        omega_eval
        x_eval
        y_eval
        theta_eval
        wl_eval
        wr_eval
        dt
    end
    
    properties(Access = private)
       V_spline_coeffs
       omega_spline_coeffs
       x_spline_coeffs
       y_spline_coeffs
       theta_spline_coeffs
       wl_spline_coeffs
       wr_spline_coeffs
    end
    
    methods
        function obj = robotTrajectory()
            %ROBOTTRAJECTORY Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        function generateInterpolators(obj)
            obj.V_spline_coeffs = spline(obj.t_eval, obj.V_eval);
            obj.omega_spline_coeffs = spline(obj.t_eval, obj.omega_eval);
            obj.x_spline_coeffs = spline(obj.t_eval, obj.x_eval);
            obj.y_spline_coeffs = spline(obj.t_eval, obj.y_eval);
            obj.theta_spline_coeffs = spline(obj.t_eval, obj.theta_eval);
            obj.wl_spline_coeffs = spline(obj.t_eval, obj.wl_eval);
            obj.wr_spline_coeffs = spline(obj.t_eval, obj.wr_eval);
        end
        
        function padSequences(obj, t_pause_)
            t_padding = zeros(floor(t_pause_/obj.dt),1);
            obj.t_eval = [(0:obj.dt:t_pause_-obj.dt)'; obj.t_eval+t_pause_; t_pause_+(obj.dt:obj.dt:t_pause_)'+max(obj.t_eval)];
            obj.V_eval = [t_padding; obj.V_eval; t_padding];
            obj.omega_eval = [t_padding; obj.omega_eval; t_padding];
            obj.x_eval = [t_padding; obj.x_eval; t_padding];
            obj.y_eval = [t_padding; obj.y_eval; t_padding];
            obj.theta_eval = [t_padding; obj.theta_eval; t_padding];
            obj.wl_eval = [t_padding; obj.wl_eval; t_padding];
            obj.wr_eval = [t_padding; obj.wr_eval; t_padding];

            
            obj.generateInterpolators();
        end
        
        function generateFigure8(obj, k_s_, k_v_, N_t)
            %GENERATEFIGURE8 Figure 8 based on coefficients for size and velocity
            %   Note that nothing is delayed yet_at_time
            v = @(t) 0.2*ones(size(t));
            s_f = 1;
            t_f = s_f/v(1);
            k_th = 2*pi/s_f;
            k_k = 15.1084;
            k_s = k_s_;
            k_v = k_v_;
            T_f = k_s/k_v*t_f;
            s = @(t) k_v/k_s*v(t).*t;
            k = @(t) k_k/k_s * sin(k_th*s(t));
            V = @(t) k_v*v(t);
            omega = @(t) k(t).*V(t);
            
            obj.dt = T_f/(N_t-1);

            obj.t_eval = linspace(0,T_f,N_t)';
            [t_out, y_out] = ode45(@(t,y)...
                [cos(y(3))*V(t); 
                 sin(y(3))*V(t); 
                 omega(t);
                 V(t) - robotModel.W2 * omega(t);
                 V(t) + robotModel.W2 * omega(t)
                 ], ...
                 linspace(0,T_f,N_t), ...
                 [0;0;0;0;0]);

            obj.V_eval = V(obj.t_eval);
            obj.omega_eval = omega(obj.t_eval);
            obj.x_eval = y_out(:,1);
            obj.y_eval = y_out(:,2);
            obj.theta_eval = y_out(:,3);
            obj.wl_eval = y_out(:,4);
            obj.wr_eval = y_out(:,5);
            
            obj.generateInterpolators();
        end
        
        function V_ = V_at_time(obj, t)
            V_ = ppval(obj.V_spline_coeffs, t);
        end
        
        function omega_ = omega_at_time(obj, t)
            omega_ = ppval(obj.omega_spline_coeffs, t);
        end
        
        function x_ = x_at_time(obj, t)
            x_ = ppval(obj.x_spline_coeffs, t);
        end
        
        function y_ = y_at_time(obj, t)
            y_ = ppval(obj.y_spline_coeffs, t);
        end
        
        function theta_ = theta_at_time(obj, t)
            theta_ = ppval(obj.theta_spline_coeffs, t);
        end
        
        function wl_ = wl_at_time(obj, t)
            wl_ = ppval(obj.wl_spline_coeffs, t);
        end

        function wr_ = wr_at_time(obj, t)
            wr_ = ppval(obj.wr_spline_coeffs, t);
        end
    end
end


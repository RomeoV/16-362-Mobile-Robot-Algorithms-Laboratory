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
        vl_eval
        vr_eval
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
       vl_spline_coeffs
       vr_spline_coeffs
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
            obj.vl_spline_coeffs = spline(obj.t_eval, obj.vl_eval);
            obj.vr_spline_coeffs = spline(obj.t_eval, obj.vr_eval);
        end
        
        function padSequences(obj, t_pause_)
            t_padding = zeros(floor(t_pause_/obj.dt),1);
            p_padding = ones(floor(t_pause_/obj.dt),1);
            t_padding = t_padding';
            p_padding = p_padding';
            obj.t_eval = [(0:obj.dt:t_pause_-obj.dt), obj.t_eval+t_pause_, t_pause_+(obj.dt:obj.dt:t_pause_)+max(obj.t_eval)];
            obj.V_eval = [t_padding, obj.V_eval, t_padding];
            obj.omega_eval = [t_padding, obj.omega_eval, t_padding];
            obj.x_eval = [p_padding*obj.x_eval(1), obj.x_eval, p_padding*obj.x_eval(end)];
            obj.y_eval = [p_padding*obj.y_eval(1), obj.y_eval, p_padding*obj.y_eval(end)];
            obj.theta_eval = [p_padding*obj.theta_eval(1), obj.theta_eval, p_padding*obj.theta_eval(end)];
            obj.vl_eval = [t_padding, obj.vl_eval, t_padding];
            obj.vr_eval = [t_padding, obj.vr_eval, t_padding];

            
            obj.generateInterpolators();
        end
        
        function generateTraj(obj, x_, y_, th_, sign_, Vmax_)
            %GENERATEFIGURE8 Figure 8 based on coefficients for size and velocity
            %   Note that nothing is delayed yet_at_time
            
            curve = cubicSpiralTrajectory.planTrajectory(x_,y_,th_,sign_);
            curve.planVelocities(Vmax_);

            obj.t_eval = curve.timeArray;
            obj.V_eval = curve.VArray;
            obj.omega_eval = curve.wArray;
            obj.x_eval = curve.poseArray(1,:);
            obj.y_eval = curve.poseArray(2,:);
            obj.theta_eval = curve.poseArray(3,:);
            obj.vl_eval = curve.vlArray;
            obj.vr_eval = curve.vrArray;
            obj.dt = obj.t_eval(end)/size(obj.t_eval,2)
            
            %obj.generateInterpolators()
            obj.padSequences(0.5);
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
        
        function vl_ = vl_at_time(obj, t)
            vl_ = ppval(obj.vl_spline_coeffs, t);
        end

        function vr_ = vr_at_time(obj, t)
            vr_ = ppval(obj.vr_spline_coeffs, t);
        end
    end
end

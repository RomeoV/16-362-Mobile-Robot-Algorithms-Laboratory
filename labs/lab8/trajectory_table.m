classdef trajectory_table < handle
    %TRAJECTORY_TABLE Precomputes many trajectories and saves them for reverse interpolation
    %   After computing each trajectory, the table will save the final
    %   position for reverse lookup to the coefficients a and b.
    %   The class also provides a static function for forwards computing the
    %   trajectory based on a, b and s_f.
    
    properties
        a_vals
        b_vals
        
        phi_vals_l
        phi_vals_r

        theta_vals_l
        theta_vals_r

        energy_vals_l
        energy_vals_r

        a_vals_l
        a_vals_r

        b_vals_l
        b_vals_r
    end
    
    methods(Static)
        function [t_,sol_] = genTrajectory(a,b,s_f)
            [t_,sol_] = ode45(@(s,state)...
                           [cos(state(3));
                            sin(state(3));
                            s.*(a+b.*s).*(s-s_f)],...
                            [0,s_f],[0;0;0]);
        end
    end
    
    methods
        function obj = trajectory_table(a_vals_, b_vals_)
            %TRAJECTORY_TABLE Constructor - reserves space
            % a_vals_ and b_vals_ are values for which trajectories will be
            % evaluated and mapped
            
            obj.a_vals = a_vals_;
            obj.b_vals = b_vals_;

            a_len = max(size(a_vals_));
            b_len = max(size(b_vals_));

            obj.phi_vals_l    = zeros(floor(a_len*b_len*0.55),1);
            obj.phi_vals_r    = zeros(floor(a_len*b_len*0.55),1);

            obj.theta_vals_l  = zeros(floor(a_len*b_len*0.55),1);
            obj.theta_vals_r  = zeros(floor(a_len*b_len*0.55),1);

            obj.energy_vals_l = zeros(floor(a_len*b_len*0.55),1);
            obj.energy_vals_r = zeros(floor(a_len*b_len*0.55),1);

            obj.a_vals_l      = zeros(floor(a_len*b_len*0.55),1);
            obj.a_vals_r      = zeros(floor(a_len*b_len*0.55),1);

            obj.b_vals_l      = zeros(floor(a_len*b_len*0.55),1);
            obj.b_vals_r      = zeros(floor(a_len*b_len*0.55),1);
        end
        
        function plotPhiTheta(obj)
            %PLOTPHITHETA Creates scatterplot to visualize table data
            
            figure
            scatter(obj.phi_vals_l,obj.theta_vals_l,'.')
            xlabel('phi');ylabel('theta')
        end
        
        function[s_, x_vals, y_vals, theta_vals] = getTrajectoryToPosition(obj, x_, y_, theta_)
          %GETTRAJECTORYTOPOSITION Provides full path to target pose
          
          phi_ = atan2(y_,x_);
          [a_u, b_u ] = obj.getCoefficientsFromTable(phi_,theta_);
          [s_, sol_] = obj.genTrajectory(a_u,b_u,1);
          r_u = norm([sol_(end,1) sol_(end,2)]);
          r_f = norm([x_ y_]);
          lambda = r_f/r_u;

          a_f = a_u/lambda^3;
          b_f = b_u/lambda^4;
          s_f = lambda;

          [s_, sol_f] = obj.genTrajectory(a_f,b_f,s_f);
          x_vals = sol_f(:,1);
          y_vals = sol_f(:,2);
          theta_vals = sol_f(:,3);
        end

        function [a,b] = getCoefficientsFromTable(obj, phi_, theta_)
          %GETCOEFFICIENTSFROMTABLE Uses 'griddata' to interpolate a and b
          energy_left_curve  = griddata(obj.phi_vals_l,obj.theta_vals_l,obj.energy_vals_l,phi_,theta_);
          energy_right_curve = griddata(obj.phi_vals_r,obj.theta_vals_r,obj.energy_vals_r,phi_,theta_);
          if energy_left_curve < energy_right_curve
            a = griddata(obj.phi_vals_l,obj.theta_vals_l,obj.a_vals_l,phi_,theta_,'cubic');
            b = griddata(obj.phi_vals_l,obj.theta_vals_l,obj.b_vals_l,phi_,theta_,'cubic');
          else
            a = griddata(obj.phi_vals_r,obj.theta_vals_r,obj.a_vals_r,phi_,theta_,'cubic');
            b = griddata(obj.phi_vals_r,obj.theta_vals_r,obj.b_vals_r,phi_,theta_,'cubic');
          end
        end
        
        function generateTable(obj)
            %GENERATETABLE Uses a and b ranges to create lookup table
            
            % Used for inserting into the tables
            running_index_l = 0;
            running_index_r = 0;

            % Start looping
            for a = obj.a_vals
                for b = obj.b_vals
                    % Generate trajectory using differential formulation and ode45
                    [s,sol] = obj.genTrajectory(a,b,1);

                    % Save to either left or right table, depending on first theta-angle
                    if sol(2,3) >= 0
                        running_index_l = running_index_l+1;
                        obj.phi_vals_l(running_index_l) = atan2(sol(end,2),sol(end,1));
                        obj.theta_vals_l(running_index_l) = sol(end,3);
                        obj.energy_vals_l(running_index_l) = sum(diff(sol(:,3)).^2);
                        obj.a_vals_l(running_index_l) = a;
                        obj.b_vals_l(running_index_l) = b;
                    else
                        running_index_r = running_index_r+1;
                        obj.phi_vals_r(running_index_r) = atan2(sol(end,2),sol(end,1));
                        obj.theta_vals_r(running_index_r) = sol(end,3);
                        obj.energy_vals_r(running_index_r) = sum(diff(sol(:,3)).^2);
                        obj.a_vals_r(running_index_r) = a;
                        obj.b_vals_r(running_index_r) = b;
                    end
                end
            end

            % Cut down the size of the arrays to the actual filled size            obj.phi_vals_l = obj.phi_vals_l(1:running_index_l);
            obj.phi_vals_r = obj.phi_vals_r(1:running_index_r);

            obj.theta_vals_l = obj.theta_vals_l(1:running_index_l);
            obj.theta_vals_r = obj.theta_vals_r(1:running_index_r);

            obj.energy_vals_l = obj.energy_vals_l(1:running_index_l);
            obj.energy_vals_r = obj.energy_vals_r(1:running_index_r);

            obj.a_vals_l = obj.a_vals_l(1:running_index_l);
            obj.a_vals_r = obj.a_vals_r(1:running_index_r);

            obj.b_vals_l = obj.b_vals_l(1:running_index_l);
            obj.b_vals_r = obj.b_vals_r(1:running_index_r);

            save('tabledata.mat','obj');
        end
    end
end

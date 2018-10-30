classdef mrplSystem
%MRPLSYSTEM Management class containing all subsystems
%   List of functions:
%   - executeTrajectory
%   - tstamp 
%   - rotateRobot
%   - findClosestSail
%   - findSails
%   - setupPlots
%   - plotData

properties
    robot
    est_robot
    ref_robot
    data_logger
    real_time_plotting
end

methods
function obj = mrplSystem()
    %MRPLSYSTEM Construct an instance of this class
    %   Detailed explanation goes here
    obj.robot = raspbot('good boi');
    obj.data_logger = logger();
    obj.data_logger.logging = true;
    obj.real_time_plotting = true;
    if obj.real_time_plotting
        obj.setupPlots();
    end
    obj.est_robot = estRobot(obj.robot.encoders.LatestMessage.Vector.X,...
             obj.robot.encoders.LatestMessage.Vector.Y);
    obj.ref_robot = refRobot(robotTrajectory());
end


function executeTrajectory(obj, trajectory)
    %EXECUTETRAJECTORY Follows a given robotTrajectory
    % The robot trajectory needs to be in robot coordinates
    if obj.real_time_plotting
        figure(1);
        subplot(1,3,1)
        xlim([min(trajectory.x_eval), max(trajectory.x_eval)]);
        ylim([min(trajectory.y_eval), max(trajectory.y_eval)]);
    end
    
    tic();
    firstIter = true;
    while toc() < max(trajectory.t_eval+2*robotModel.tdelay)
      if firstIter
          tic();
          firstIter = false;
          obj.ref_robot.trajectory_starting_time = toc();
          tstamp = obj.getLatestRobotTime();
          current_time = toc();
          pause(0.001);
      end
      last_time = current_time;
      current_time = toc();
      dt = current_time - last_time;
      %disp(dt + " : " + est_robot.dtime)
      encoder_l = obj.robot.encoders.LatestMessage.Vector.X;
      encoder_r = obj.robot.encoders.LatestMessage.Vector.Y;

      last_tstamp = tstamp;
      tstamp = obj.getLatestRobotTime();
      d_tstamp = tstamp-last_tstamp;
      %disp("Matlab dt: " + dt + " - Robot dt: " + d_tstamp)
      %tstamp = current_time;

      obj.est_robot.updateEstimation(d_tstamp,encoder_l,encoder_r);

      delayed_time = max(0,current_time-robotModel.tdelay);
      %pose_ref_in_rf = ref_robot.getReferencePoseAtTime(current_time-robotModel.tdelay);
      pose_ref_in_rf = pose(trajectory.x_at_time(delayed_time),...
          trajectory.y_at_time(delayed_time),...
          trajectory.theta_at_time(delayed_time));

      pose_ref_in_wf = pose(pose.matToPoseVec(obj.est_robot.getPose().bToA()*pose_ref_in_rf.bToA()));

      pose_est = obj.est_robot.getPose();

      vl_ffd = trajectory.vl_at_time(delayed_time);

      vr_ffd = trajectory.vr_at_time(delayed_time);

      [vl_control, vr_control] = controller.getControlInput(pose_ref_in_wf, pose_est,...
          vl_ffd, vr_ffd,...
          trajectory.V_at_time(current_time-robotModel.tdelay),obj.data_logger);

        obj.data_logger.log_data(pose_est.x(), pose_est.y(), pose_est.th(), encoder_l, encoder_r, current_time, pose_ref_in_wf.x(), pose_ref_in_wf.y(), pose_ref_in_wf.th());

          % Send control to plant
        if isnan(vl_control) || isnan(vr_control)
            obj.robot.sendVelocity(0, 0);
        else
            obj.robot.sendVelocity(vl_control, vr_control); 
        end

        if obj.real_time_plotting
          obj.plotData();
          drawnow();
        end
      pause(0.005);
    end % while loop
end % function executeTrajectory

function tstamp = getLatestRobotTime(obj)
    %GETLATESTROBOTTIME returns time as given by last robot message
    tstamp = double(obj.robot.encoders.LatestMessage.Header.Stamp.Sec)+double(obj.robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
end

function rotateRobot(obj, th)
    %ROTATEROBOT rotates the robot with given angle in rad
    L = abs(th)*robotModel.W2;
    t = linspace(0,4);
    omega = zeros(size(t));
    for i = 1:max(size(t))
        omega(i) = trapezoidalVelocityProfile(t(i),pi/50,L,sign(th));
    end
    t_init = obj.getLatestRobotTime();
    t_latest = t_init;
    first_loop = true;
    while(t_latest - t_init < t(end))
        if first_loop
            t_init = obj.getLatestRobotTime();
            first_loop = false;
        end
        t_latest = obj.getLatestRobotTime();
        v = interp1(t,omega,t_latest-t_init,'Spline');
        obj.robot.sendVelocity(-v,v);
        pause(0.05);
    end
    obj.robot.stop();
end

function sail = findClosestSail(obj)
    %FINDCLOSESTSAIL Returns closest sail as vector (x,y,th)
    sails = obj.findSails(1);
    if size(sails,2) > 0
        sails = sails(1:3,:);
        sails(3,:) = deg2rad(sails(3,:));

        [~,idx_min] = min(mean(sails(1:2,:)));

        sail = sails(:,idx_min);

        % turn so it's facing 'away'
        if (sail(2) < 0 && sail(3) > 0) || (sail(2) > 0 && sail(3) < 0)
            sail(2) = -1*sail(2);
        end
        
        if obj.real_time_plotting
           figure(1)
           subplot(1,3,1); hold on;
           scatter(sail(1), sail(2),'rx'); hold off;
        end
    else
        sail = [];
    end
end

function sails = findSails(obj, maxNumOfSails)
  %FINDSAILS tries to find sails for 5 seconds or until maxNumOfSails
  %reached. Also has different cutofff distances (see code)
  close = true;
if close
    min_r = 0.02;
    max_r = 0.6;
else
    min_r = 0.08;
    max_r = 1.5;
end

sails = [];

tic()
firstIter = true;
found_sail = 0;

while toc() < 5 && found_sail<maxNumOfSails
  if firstIter
      tic();
      firstIter = false;
  end
  %plot points around it
  
  %Get r_values
  r_values = circshift(obj.robot.laser.LatestMessage.Ranges,robotModel.laserOffset);
  %Remove bad values
  th = linspace(1,360,360)';
  if close
     close_pallet = th<45 | th > 315;
     th = th(close_pallet);
     r_values = r_values(close_pallet);
  end
  goodones = r_values>min_r & r_values<max_r;
  r_values = r_values(goodones);
  th = th(goodones);
  
  
  %Find Sail
  for i = 1:size(r_values,1)
        if r_values(i)>min_r
            arc_length = r_values(i)*deg2rad(1);
            pm_sail = floor(0.065/arc_length);
            pm_th = rad2deg(0.065/r_values(i));
            indices = get_ca_ij(r_values,th,th(i)-pm_th,th(i)+pm_th);
            sail_points = r_values(indices);
            theta = deg2rad(th(indices));
            x = cos(theta).* sail_points;
            y = sin(theta).*sail_points;
            
            center_x = mean(x);
            center_y = mean(y);
            
            x = x - center_x;
            y = y - center_y;
            
            %Inertia
            
            Ixx = x' * x;
            Iyy = y' * y;
            Ixy = -x' * y;
            Inertia = [Ixx Ixy;Ixy Iyy] /(size(th,1));
            lambda = eig(Inertia); 
            lambda = real(sqrt(lambda)*1000.0);
            %disp(min(lambda))
            if ~isempty(lambda) && min(lambda)<1.3 && min(lambda)>0
                sail_th = rad2deg(atan2(2*Ixy,Iyy-Ixx)/2);
                %disp(center_x + " : " + center_y + " : " + sail_th + " : " + r_values(i))
                %Check if near other points\
                if size(sails,2)>0
                    new_point = true;
                    for p = 1:size(sails,2)
                        sx = sails(1,p);
                        sy = sails(2,p);
                        sth = sails(3,p);
                        n = sails(4,p);
                        dist = sqrt((sx - center_x)^2 +(sy-center_y)^2);
                        if dist<0.13
                            new_point = false; 
                            sails(1,p) = ((sx*n)+center_x)/(n+1);
                            sails(2,p) = ((sy*n)+center_y)/(n+1);
                            sails(3,p) = ((sth*n)+sail_th)/(n+1);
                            sails(4,p) = n+1;
                            found_sail = found_sail +1;
                        end
                        
                    end
                    if new_point
                        if (sqrt((center_x)^2 +(center_y)^2)>min_r)
                            sails = horzcat(sails,[center_x;center_y;sail_th;1]); 
                            found_sail = found_sail +1;
                        end
                    end
                else
                    if (sqrt((center_x)^2 +(center_y)^2)>min_r)
                        sails = horzcat(sails,[center_x;center_y;sail_th;1]); 
                        found_sail = found_sail +1;
                    end
                end                
            end % end of if regarding lambda
         end % end of if regarding range
  end % end of for-loop over range values
  pause(0.05)
end % end of while loop for finding sails
end % end function findSails

function setupPlots(obj)
    %SETUPPLOTS Subplot setup for trajectory+sails, error_theta, error_x_y_th
    figure(1)
    subplot(1,3,1);
    hold off;
    plot([0],[0]); hold on;
    scatter(0,0,'go'); hold off;
    xlabel('x');
    ylabel('y');
    daspect([1 1 1]);
    xlim([-1,1]);
    ylim([-1,1]);
    
    subplot(1,3,2);
    hold off;
    plot([0],[0]);
    xlabel('time');
    ylabel('theta error');
    
    subplot(1,3,3);
    hold off;
    plot([0],[0]);
    xlabel('time');
    ylabel('errors (x,y,theta)');
end

function plotData(obj)
    %SETUPPLOTS Subplot setup for trajectory+sails, error_theta, error_x_y_th
    figure(1);
    subplot(1,3,1)
    plot(obj.data_logger.x_est_data,obj.data_logger.y_est_data);
    hold on
    plot(obj.data_logger.x_data,obj.data_logger.y_data,'--');
    hold off;

    subplot(1,3,2)
    plot(obj.data_logger.theta_est_data); hold on;
    plot(obj.data_logger.theta_data); hold off;

    subplot(1,3,3)
    plot(obj.data_logger.error_x_data); hold on;
    plot(obj.data_logger.error_y_data);
    plot(obj.data_logger.error_theta_data); hold off;
end % function plotData

end % end of methods section
end % end of mrpsSystem class

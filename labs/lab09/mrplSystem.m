classdef mrplSystem
%MRPLSYSTEM Management class containing all subsystems
%   List of functions:
%   - pickUpClosestSail
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
    obj.robot.startLaser();
end

function redirectToClosestSail(obj)
    
    r_values = circshift(obj.robot.laser.LatestMessage.Ranges,robotModel.laserOffset);
    %Remove bad values
    th = linspace(1,360,360)';
    goodones = r_values>0.08 & r_values<0.3;
    r_values = r_values(goodones);
    th = th(goodones);
    goodth1 = th<45 | th>315;
    r_values = r_values(goodth1);
    th = th(goodth1);
    
    close_ones = r_values<min(r_values)+0.1;
    r_values=r_values(close_ones);
    th = th(close_ones);
    
    
    for i = 1:size(th,1)
        
        if th(i) > 180
           th(i) = th(i)-360; 
        end
        
    end
    
    disp(min(th)+ " : " + max(th))
    theta = (min(th)+max(th))/2
    theta = deg2rad(theta);
    
    obj.rotateRobot(1*theta);
end

function pickUpClosestSail(obj)
    %% Generate Trajectory to Sail
    sail_in_rf = obj.findClosestSail()
    
    sail_in_rf(3) = sail_in_rf(3)*0.8
    
    pause()
    
    fork_offset = pose(-0.15,0,0);
    fork_goal_pose_in_robot_frame = pose(pose.matToPoseVec(...
        pose(sail_in_rf).bToA()*fork_offset.bToA()));
    trajectory = robotTrajectory();
    trajectory.generateTraj(fork_goal_pose_in_robot_frame.x(),...
        fork_goal_pose_in_robot_frame.y(),...
        fork_goal_pose_in_robot_frame.th(),...
        1,0.2);

    %sys.plotTrajectory(trajectory);
    obj.robot.forksDown();
    
    obj.executeTrajectory(trajectory)
    
    obj.robot.stop();
    
    pause()

    obj.redirectToClosestSail();
    obj.robot.stop();
    pause();
    obj.redirectToClosestSail();
    
    obj.robot.forksDown();
    
    obj.robot.stop();
    
    %obj.redirectToClosestSail();

    %% Pick up the sail
    pause(1);
    obj.moveForwards(0.2);
    obj.robot.stop();
    obj.robot.forksUp();
    pause(1);
    obj.rotateRobot(pi);
    obj.robot.sendVelocity(0.2,0.2);
    pause(1);
end

function plotTrajectory(obj, trajectory)
    figure(1);
    hold on;
    %xlim([min(trajectory.x_eval), max(trajectory.x_eval)]);
    %ylim([min(trajectory.y_eval), max(trajectory.y_eval)]);

    N = max(size(trajectory.x_eval));
    x_plt = zeros(1,N);
    y_plt = zeros(1,N);
    for p = 1:N
        goal_pose_in_wf = pose(pose.matToPoseVec(...
            obj.est_robot.getPose().bToA()*pose(trajectory.x_eval(p),trajectory.y_eval(p),0).bToA()));
        x_plt(p) = goal_pose_in_wf.x();
        y_plt(p) = goal_pose_in_wf.y();
    end
    hold on;
    plot(x_plt,y_plt); hold off;
end

function executeTrajectory(obj, trajectory)
    %EXECUTETRAJECTORY Follows a given robotTrajectory
    % The robot trajectory needs to be in robot coordinates
    if obj.real_time_plotting
        obj.plotTrajectory(trajectory);
    end
    
    initial_robot_frame = obj.est_robot.getPose();
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

      pose_ref_in_wf = pose(pose.matToPoseVec(initial_robot_frame.bToA()*pose_ref_in_rf.bToA()));

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
    t = linspace(0,sqrt(4*L/(pi/50)));
    omega = zeros(size(t));
    rot_pose_i = obj.est_robot.getPose()
    
    th_ref = 0;
    
    for i = 1:max(size(t))
        omega(i) = trapezoidalVelocityProfile(t(i),pi/50,L,sign(th));
    end
    t_init = obj.getLatestRobotTime();
    t_latest = t_init;
    first_loop = true;
    while(t_latest - t_init < t(end))
        if first_loop
            t_init = obj.getLatestRobotTime();
            t_latest = t_init;
            first_loop = false;
        end
        encoder_l = obj.robot.encoders.LatestMessage.Vector.X;
        encoder_r = obj.robot.encoders.LatestMessage.Vector.Y;
        last_tstamp = t_latest;
        t_latest = obj.getLatestRobotTime();
        d_tstamp = t_latest-last_tstamp;

        obj.est_robot.updateEstimation(d_tstamp,encoder_l,encoder_r);
        
        v = interp1(t,omega,t_latest-t_init,'Spline');
        
        th_ref = th_ref+v*d_tstamp/robotModel.W2;
        rot_pose = obj.est_robot.getPose();
        
        error_th = (rot_pose_i.th+th_ref) - rot_pose.th;
        %disp(th_ref)
        Kp = 0.2;
        
        vel =(v+Kp*error_th);
        if abs(vel)<0.005
            vel = 0.005*sign(vel);
        end
        obj.robot.sendVelocity(-vel,vel);

        if obj.real_time_plotting
          obj.plotData();
          drawnow();
        end

        pause(0.05);
    end
    obj.robot.stop();
end

function moveForwards(obj, L)
    %ROTATEROBOT rotates the robot with given angle in rad
    t = linspace(0,sqrt(4*abs(L)/(pi/50)));
    speed = zeros(size(t));
    for i = 1:max(size(t))
        speed(i) = trapezoidalVelocityProfile(t(i),pi/50,L,sign(L));
    end
    t_init = obj.getLatestRobotTime();
    t_latest = t_init;
    first_loop = true;
    while(t_latest - t_init < t(end))
        if first_loop
            t_init = obj.getLatestRobotTime();
            t_latest = t_init;
            first_loop = false;
        end
        encoder_l = obj.robot.encoders.LatestMessage.Vector.X;
        encoder_r = obj.robot.encoders.LatestMessage.Vector.Y;
        last_tstamp = t_latest;
        t_latest = obj.getLatestRobotTime();
        d_tstamp = t_latest-last_tstamp;

        obj.est_robot.updateEstimation(d_tstamp,encoder_l,encoder_r);

        v = interp1(t,speed,t_latest-t_init,'Spline');
        obj.robot.sendVelocity(v,v);

        if obj.real_time_plotting
          obj.plotData();
          drawnow();
        end

        pause(0.05);
    end
    obj.robot.stop();
end

function sail = findClosestSail(obj)
    range_image = rangeImage(obj.robot.laser.LatestMessage.Ranges);
    [sails, walls] = range_image.findSailsAndWalls();
    assert(~isempty(sails),'No sail found!');
    [~,idx] = min(norm(sails(1:2,:),1));
    sail = sails(:,idx);
    
    if obj.real_time_plotting
        range_image.plotXvsY();
        range_image.plotSails(sails);
        range_image.plotWalls(walls);
        
        sail_pose_in_rf = pose(sail);
        sail_pose_in_wf = pose(pose.matToPoseVec(...
            obj.est_robot.getPose().bToA()*sail_pose_in_rf.bToA()));
       figure(1)
       hold on;
       quiver(sail_pose_in_wf.x(), sail_pose_in_wf.y(),...
           -cos(sail_pose_in_wf.th()),-sin(sail_pose_in_wf.th()),0.05,'filled','-.xr','LineWidth',2); hold off;
    end
end

function setupPlots(obj)
    %SETUPPLOTS Subplot setup for trajectory+sails, error_theta, error_x_y_th
    figure(1)
    hold off;
    plot([0],[0]); hold on;
    scatter(0,0,'go'); hold off;
    xlabel('x');
    ylabel('y');
    daspect([1 1 1]);
    xlim([-1,1]);
    ylim([-1,1]);
    
    figure(2)
    subplot(1,2,1);
    hold off;
    plot([0],[0]);
    xlabel('time');
    ylabel('theta error');
    
    subplot(1,2,2);
    hold off;
    plot([0],[0]);
    xlabel('time');
    ylabel('errors (x,y,theta)');
end

function plotData(obj)
    %SETUPPLOTS Subplot setup for trajectory+sails, error_theta, error_x_y_th
    figure(1);
    hold on;
    plot(obj.data_logger.x_est_data,obj.data_logger.y_est_data);
    hold on
    plot(obj.data_logger.x_data,obj.data_logger.y_data,'--');
    hold off;

    figure(2);
    subplot(1,2,1)
    plot(obj.data_logger.theta_est_data); hold on;
    plot(obj.data_logger.theta_data); hold off;

    subplot(1,2,2)
    plot(obj.data_logger.error_x_data); hold on;
    plot(obj.data_logger.error_y_data);
    plot(obj.data_logger.error_theta_data); hold off;
end % function plotData

end % end of methods section
end % end of mrpsSystem class

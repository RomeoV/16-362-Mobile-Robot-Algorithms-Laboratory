%% Setup workspace
clear; close all;

trajectory = robotTrajectory();
trajectory.generateFigure8(1,.5,100);
%trajectory.padSequences(2.5);

robot = raspbot('sim');

K_p = 1.3;

wl_init = robot.encoders.LatestMessage.Vector.X;
wr_init = robot.encoders.LatestMessage.Vector.Y;
wl_data = [];
wr_data = [];

x_est = 0;
y_est = 0;
theta_est = 0;

x_est_data = [];
y_est_data = [];
theta_est_data = [];

error_l_data = [];
error_r_data = [];

encoder_l = 0;
encoder_r = 0;

t_data = [];

firstIter = true;
current_time = 0;

tic();
while toc() < max(trajectory.t_eval+robotModel.tdelay)
    if firstIter
        tic();
        firstIter = false;
    end
    last_time = current_time;
    current_time = toc();
    dt = current_time - last_time;
    [vl_ffd, vr_ffd] = robotModel.VwTovlvr(trajectory.V_at_time(current_time), trajectory.omega_at_time(current_time));
    
    encoder_last_l = encoder_l;
    encoder_last_r = encoder_r;
    
    encoder_l = robot.encoders.LatestMessage.Vector.X-wl_init;
    encoder_r = robot.encoders.LatestMessage.Vector.Y-wr_init;
    
    % State estimator
    vl_est = (encoder_l-encoder_last_l)/dt;
    vr_est = (encoder_r-encoder_last_r)/dt;
    [V_est, omega_est] = robotModel.vlvrToVw(vl_est, vr_est);
    x_est = x_est + cos(theta_est)*V_est*dt/2;
    y_est = y_est + sin(theta_est)*V_est*dt/2;
    theta_est = theta_est + omega_est*dt;
    x_est = x_est + cos(theta_est)*V_est*dt/2;
    y_est = y_est + sin(theta_est)*V_est*dt/2;
    
    % Record data
    x_est_data = [x_est_data x_est];
    y_est_data = [y_est_data y_est];
    theta_est_data = [theta_est_data theta_est];
    wl_data = [wl_data encoder_l];
    wr_data = [wr_data encoder_r];
    t_data = [t_data current_time];
    
    % Controller
    error_l = trajectory.wl_at_time(current_time-robotModel.tdelay) - encoder_l;
    error_r = trajectory.wr_at_time(current_time-robotModel.tdelay) - encoder_r;
    error_rel = [cos(theta_est) -sin(theta_est); sin(theta_est) cos(theta_est)]*[trajectory.x_at_time(current_time-robotModel.tdelay) - x_est;trajectory.y_at_time(current_time-robotModel.tdelay) - y_est];
    error_l_data = [error_l_data error_rel(1)];
    error_r_data = [error_r_data error_rel(2)];
%     
%     vl_fb = K_p*error_l;
%     vr_fb = K_p*error_r;
    V_control = 1.4*error_rel(1);
    vw_control = 1.4*error_rel(2);
    error_dist = error_l^2+error_r^2;
    error_theta = trajectory.theta_at_time(current_time-robotModel.tdelay);
    [vl_fb, vr_fb] = robotModel.VwTovlvr(V_control, vw_control);
%     vl_fb = vl_control;
%     vr_fb = vr_control;
    
    [vl_sat, vr_sat] = robotModel.limitWheelVelocities([vl_ffd+vl_fb; vr_ffd+vr_fb]);
    
    % Send control to plant
    robot.sendVelocity(vl_sat, vr_sat);
    pause(0.05);
end
robot.stop();
robot.shutdown();
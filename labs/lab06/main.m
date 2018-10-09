%% Setup workspace
clear;
close all;
robot = raspbot('hamilton');

trajectory = robotTrajectory();
trajectory.generateTraj(0.3048,0.3048,0,1,0.2);
%trajectory.padSequences(2.5);

K_p = 1.3;
pid = 1;
logging = true;

wl_init = robot.encoders.LatestMessage.Vector.X;
wr_init = robot.encoders.LatestMessage.Vector.Y;
wl_data = [];
wr_data = [];
wh_offset = 0.98;

x_est = 0;
y_est = 0;
theta_est = 0;

x_est_data = [];
y_est_data = [];
theta_est_data = [];

error_x_data = [];
error_y_data = [];
error_theta_data = [];

encoder_l = 0;
encoder_r = 0;

x_data = [];
y_data = [];

t_data = [];

firstIter = true;
current_time = 0;
tic()
while toc() < max(trajectory.t_eval+robotModel.tdelay)
    if firstIter
        tic();
        firstIter = false;
    end
    last_time = current_time;
    current_time = toc();
    
    dt = current_time - last_time;
    vl_ffd = trajectory.vl_at_time(current_time);
    vr_ffd = trajectory.vr_at_time(current_time);
    
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
    
    
    x_data = [x_data trajectory.x_at_time(current_time)];
    y_data = [y_data trajectory.y_at_time(current_time)];
    
    % Controller
    error_x = trajectory.x_at_time(current_time-robotModel.tdelay) - x_est;
    error_y = trajectory.y_at_time(current_time-robotModel.tdelay) - y_est;

    error_rel = [cos(theta_est) sin(theta_est); -sin(theta_est) cos(theta_est)]*[error_x;error_y];
    error_theta = trajectory.theta_at_time(current_time-robotModel.tdelay) - theta_est;
    
    % Record data
    if logging
    x_est_data = [x_est_data x_est];
    y_est_data = [y_est_data y_est];
    theta_est_data = [theta_est_data theta_est];
    wl_data = [wl_data encoder_l];
    wr_data = [wr_data encoder_r];
    t_data = [t_data current_time];
    error_x_data = [error_x_data error_rel(1)];
    error_y_data = [error_y_data error_rel(2)];
    error_theta_data = [error_theta_data error_theta];
    end

    
    tau = 1.2;
    k_x = 1/tau;
    if (trajectory.V_at_time(current_time-robotModel.tdelay) < .005)
        k_y = 0;
    else
        k_y = 2/(tau^2*abs(trajectory.V_at_time(current_time-robotModel.tdelay)));
    end
    k_theta = 1/tau;
    
    error_V = k_x*error_rel(1);
    error_omega = k_y*error_rel(2)+k_theta*error_theta;
    [vl_fb, vr_fb] = robotModel.VwTovlvr(error_V, error_omega);


%     vl_fb = K_p*error_l;
%     vr_fb = K_p*error_r;
%     vl_control = 0.4*error_l;
%     vr_control = 0.4*error_r;
%     error_dist = error_l^2+error_r^2;
%     error_theta = trajectory.theta_at_time(current_time-robotModel.tdelay);
    %[vl_fb, vr_fb] = robotModel.VwTovlvr(vl_control, vr_control);
    
    [vl_sat, vr_sat] = robotModel.limitWheelVelocities([vl_ffd+pid*vl_fb; vr_ffd+pid*vr_fb]);
    
    % Send control to plant
    if isnan(vl_sat) || isnan(vl_sat)
        robot.sendVelocity(0, 0);
    else
        robot.sendVelocity(vl_sat, vr_sat*wh_offset); 
    end
    
    plot_graphs
    drawnow()
    pause(0.005);
end

pause(1)

trajectory = robotTrajectory();
trajectory.generateTraj(-.6096,-.6096,-pi/2,1,0.2);
%trajectory.padSequences(2.5);

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

error_x_data = [];
error_y_data = [];
error_theta_data = [];

encoder_l = 0;
encoder_r = 0;

x_data = [];
y_data = [];

t_data = [];

firstIter = true;
current_time = 0;
tic()
figure()
while toc() < max(trajectory.t_eval+robotModel.tdelay)
    if firstIter
        tic();
        firstIter = false;
    end
    last_time = current_time;
    current_time = toc();
    
    dt = current_time - last_time;
    vl_ffd = trajectory.vl_at_time(current_time);
    vr_ffd = trajectory.vr_at_time(current_time);
    
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
    
    
    x_data = [x_data trajectory.x_at_time(current_time)];
    y_data = [y_data trajectory.y_at_time(current_time)];
    
    % Controller
    error_x = trajectory.x_at_time(current_time-robotModel.tdelay) - x_est;
    error_y = trajectory.y_at_time(current_time-robotModel.tdelay) - y_est;

    error_rel = [cos(theta_est) sin(theta_est); -sin(theta_est) cos(theta_est)]*[error_x;error_y];
    error_theta = trajectory.theta_at_time(current_time-robotModel.tdelay) - theta_est;
    
    % Record data
    if logging
    x_est_data = [x_est_data x_est];
    y_est_data = [y_est_data y_est];
    theta_est_data = [theta_est_data theta_est];
    wl_data = [wl_data encoder_l];
    wr_data = [wr_data encoder_r];
    t_data = [t_data current_time];
    error_x_data = [error_x_data error_rel(1)];
    error_y_data = [error_y_data error_rel(2)];
    error_theta_data = [error_theta_data error_theta];
    end

    
    tau = 1.2;
    k_x = 1/tau;
    if (trajectory.V_at_time(current_time-robotModel.tdelay) < .005)
        k_y = 0;
    else
        k_y = 2/(tau^2*abs(trajectory.V_at_time(current_time-robotModel.tdelay)));
    end
    k_theta = 1/tau;
    
    error_V = k_x*error_rel(1);
    error_omega = k_y*error_rel(2)+k_theta*error_theta;
    [vl_fb, vr_fb] = robotModel.VwTovlvr(error_V, error_omega);


%     vl_fb = K_p*error_l;
%     vr_fb = K_p*error_r;
%     vl_control = 0.4*error_l;
%     vr_control = 0.4*error_r;
%     error_dist = error_l^2+error_r^2;
%     error_theta = trajectory.theta_at_time(current_time-robotModel.tdelay);
    %[vl_fb, vr_fb] = robotModel.VwTovlvr(vl_control, vr_control);
    
    [vl_sat, vr_sat] = robotModel.limitWheelVelocities([vl_ffd+pid*vl_fb; vr_ffd+pid*vr_fb]);
    
    % Send control to plant
    if isnan(vl_sat) || isnan(vl_sat)
        robot.sendVelocity(0, 0);
    else
        robot.sendVelocity(vl_sat, vr_sat*wh_offset); 
    end
    
    plot_graphs
    drawnow()
    pause(0.005);
end

pause(1)

trajectory = robotTrajectory();
trajectory.generateTraj(-0.3048,0.3048,pi/2,1,0.2);
%trajectory.padSequences(2.5);


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

error_x_data = [];
error_y_data = [];
error_theta_data = [];

encoder_l = 0;
encoder_r = 0;

x_data = [];
y_data = [];

t_data = [];

firstIter = true;
current_time = 0;
tic()
figure()
while toc() < max(trajectory.t_eval+robotModel.tdelay)
    if firstIter
        tic();
        firstIter = false;
    end
    last_time = current_time;
    current_time = toc();
    
    dt = current_time - last_time;
    vl_ffd = trajectory.vl_at_time(current_time);
    vr_ffd = trajectory.vr_at_time(current_time);
    
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
    
    
    x_data = [x_data trajectory.x_at_time(current_time)];
    y_data = [y_data trajectory.y_at_time(current_time)];
    
    % Controller
    error_x = trajectory.x_at_time(current_time-robotModel.tdelay) - x_est;
    error_y = trajectory.y_at_time(current_time-robotModel.tdelay) - y_est;

    error_rel = [cos(theta_est) sin(theta_est); -sin(theta_est) cos(theta_est)]*[error_x;error_y];
    error_theta = trajectory.theta_at_time(current_time-robotModel.tdelay) - theta_est;
    
    % Record data
    if logging
    x_est_data = [x_est_data x_est];
    y_est_data = [y_est_data y_est];
    theta_est_data = [theta_est_data theta_est];
    wl_data = [wl_data encoder_l];
    wr_data = [wr_data encoder_r];
    t_data = [t_data current_time];
    error_x_data = [error_x_data error_rel(1)];
    error_y_data = [error_y_data error_rel(2)];
    error_theta_data = [error_theta_data error_theta];
    end

    
    tau = 1.2;
    k_x = 1/tau;
    if (trajectory.V_at_time(current_time-robotModel.tdelay) < .005)
        k_y = 0;
    else
        k_y = 2/(tau^2*abs(trajectory.V_at_time(current_time-robotModel.tdelay)));
    end
    k_theta = 1/tau;
    
    error_V = k_x*error_rel(1);
    error_omega = k_y*error_rel(2)+k_theta*error_theta;
    [vl_fb, vr_fb] = robotModel.VwTovlvr(error_V, error_omega);


%     vl_fb = K_p*error_l;
%     vr_fb = K_p*error_r;
%     vl_control = 0.4*error_l;
%     vr_control = 0.4*error_r;
%     error_dist = error_l^2+error_r^2;
%     error_theta = trajectory.theta_at_time(current_time-robotModel.tdelay);
    %[vl_fb, vr_fb] = robotModel.VwTovlvr(vl_control, vr_control);
    
    [vl_sat, vr_sat] = robotModel.limitWheelVelocities([vl_ffd+pid*vl_fb; vr_ffd+pid*vr_fb]);
    
    % Send control to plant
    if isnan(vl_sat) || isnan(vl_sat)
        robot.sendVelocity(0, 0);
    else
        robot.sendVelocity(vl_sat, vr_sat*wh_offset); 
    end
    
    plot_graphs
    drawnow()
    pause(0.005);
end

pause(1)

robot.stop();
robot.shutdown();
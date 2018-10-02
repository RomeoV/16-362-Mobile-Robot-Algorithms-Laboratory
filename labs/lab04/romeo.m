clear; 

robot = raspbot('hamilton');


%% Setup Controller parameters
K_p = 3.5;
K_d = 0.25;
K_i = 0.0;

dist_ref = 0.1;
dist_tolerance = 1e-4;

saturation = 0.3;

%% Setup initial robot parameters

encoder_l_initial = robot.encoders.LatestMessage.Vector.X;
encoder_r_initial = robot.encoders.LatestMessage.Vector.Y;

N_timesteps = 200;
dist_travelled_l = dist_ref*ones(1,N_timesteps);
dist_travelled_r = dist_ref*ones(1,N_timesteps);
dist_travelled = dist_ref*ones(1,N_timesteps);
dist_travelled(1) = 0; dist_travelled_l(1) = 0; dist_travelled_r(1) = 0;
dist_error = zeros(1,N_timesteps);
dist_error_d = zeros(1,N_timesteps);
dist_integrator = zeros(1,N_timesteps);

u_pid = zeros(1,N_timesteps);
u = zeros(1,N_timesteps);


timestep = zeros(1,N_timesteps);


idx = 1;
first_loop = true;

tic();
last_time = 0;
while abs(dist_ref-dist_travelled(idx)) > dist_tolerance
    if ~first_loop 
        idx = idx+1;
    else
        first_loop = false;
    end
    % State estimation (distance)
    dist_travelled_l(idx) = robot.encoders.LatestMessage.Vector.X - encoder_l_initial;
    dist_travelled_r(idx) = robot.encoders.LatestMessage.Vector.Y - encoder_r_initial;
    dist_travelled(idx) = (dist_travelled_l(idx)+dist_travelled_r(idx))/2;
    
    % Plot a bunch of stuff
    if mod(idx,30)==0
    plot(dist_travelled); hold on; % Plot distances travelled
    plot(dist_integrator);
    plot([0 N_timesteps],[dist_ref dist_ref],'--'); % Plot distance reference
    plot(dist_error); plot([0 N_timesteps],[0 0], '--'); % Plot error and error reference
    plot(dist_error_d);
    hold off; %legend('Dist L', 'Dist R', 'Dist M', 'Dist Ref', 'Error', 'Error Ref');
    end
    
    % Gain and control calculation
        % Proportional term
    dist_error(idx) = dist_ref-dist_travelled(idx);
    
        % Derivative term
    timestep(idx) = toc();
    if idx>1
    dt = timestep(idx) - timestep(idx-1);
    dist_error_d(idx) = (dist_error(idx)-dist_error(idx-1))/dt;
    end
    
        % PID term
    u_pid(idx) = K_p * dist_error(idx) + K_d * dist_error_d(idx);
    
    % Saturation
    if abs(u_pid(idx)) > saturation
        u_pid(idx) = 0.3*sign(u_pid(idx));
    end
    
    % Command
    u(idx) = u_pid(idx);
    robot.sendVelocity(u(idx),u(idx));
    dist_integrator(idx) = trapz(timestep(1:idx),u(1:idx));
    
    if mod(idx,30)==0
    drawnow();
    end
    pause(0.01)
end
robot.stop()
sprintf('Terminated in %0.2g seconds', toc())

plot(dist_travelled); hold on; % Plot distances travelled
plot([0 N_timesteps],[dist_ref dist_ref],'--'); % Plot distance reference
plot(dist_error); plot([0 N_timesteps],[0 0], '--'); % Plot error and error reference
plot(dist_error_d);
hold off; %legend('Dist L', 'Dist R', 'Dist M', 'Dist Ref', 'Error', 'Error Ref');

function u_ref = u_ramp(t, final_distance)
    a_max = 3*0.25;
    v_max = 0.25;
    t_ramp = v_max/a_max;
    t_f = final_distance/v_max + t_ramp;
    if t_ramp > t_f/2
        t_ramp = t_f/2;
    end
    
    if t < t_ramp
        u_ref =  t*a_max;
    elseif t_ramp < t && t < t_f-t_ramp
        u_ref = v_max;
    elseif (tf-t) < t_ramp
        u_ref = a_max*(t_f-t);
    else
        u_ref = 0;
    end
 end
clear;
robot = raspbot('hamilton');

%% Set up constants
s_f = 1;
V = .2;
t_f = s_f/V;
k_k = 15.1084;
k_s = 3;
k_theta = 2*pi/s_f;
T_f = k_s*t_f;
n_T = 10*T_f;
d_T = (T_f-0)/(n_T);
W = 0.09;

%% Set up differential equation for movement
f = @(t, state) [
V*cos(state(3));
V*sin(state(3));
k_k/k_s*sin(k_theta*V*t/k_s)*V
];

%% Solve and plot ODE
[t_out, state_out] = ode45(f,linspace(0,T_f,n_T+1),[0,0,0]);


omega = (state_out(2,3)-state_out(1,3))/(d_T);
v_l = V - W/2 * omega;
v_r = V + W/2 * omega;
robot.sendVelocity(V,V);
pause(d_T)

for i = 2:n_T
    tic()
    omega = (state_out(i+1,3)-state_out(i-1,3))/(2*d_T);
    v_l = V - W/2 * omega;
    v_r = V + W/2 * omega;
    robot.sendVelocity(v_l,v_r);
    
    while toc()<d_T
    end
end

tic()
omega = (state_out(end,3)-state_out(end-1,3))/(d_T);
v_l = V - W/2 * omega;
v_r = V + W/2 * omega;
robot.sendVelocity(V,V);
while toc()<d_T
end
robot.stop();

    
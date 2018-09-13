% THIS NEEDS TO BE FINISHED

clear;

%% Set up constants
s_f = 1;
V = .2;
t_f = s_f/V;
k_k = 15.1084;
k_s = 3;
k_theta = 2*pi/s_f;
T_f = k_s*t_f;
n_T = 20*T_f;
d_T = (T_f-0)/(n_T);
W = 0.09;

x = 0;
y = 0;
theta = 0;

%% Set up differential equation for movement
f = @(t, state) [
V*cos(state(3));
V*sin(state(3));
k_k/k_s*sin(k_theta*V*t/k_s)*V
];

%% Solve and plot ODE
[t_out, state_out] = ode45(f,linspace(0,T_f,n_T+1),[0,0,0]);


for i = 2:n_T
    omega = (state_out(i+1,3)-state_out(i-1,3))/(2*d_T);
    v_l = V - W/2 * omega;
    v_r = V + W/2 * omega;
    %robot.sendVelocity(v_l,v_r);
    pause(0.05);
end
%robot.sendVelocity(V,V);
pause(0.05)
%robot.stop();

    
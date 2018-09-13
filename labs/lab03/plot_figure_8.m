clear;

%% Set up constants
s_f = 1;
V = .2;
t_f = s_f/V;
k_k = 15.1084;
k_s = 3;
k_theta = 2*pi/s_f;
T_f = k_s*t_f;

%% Set up differential equation for movement
f = @(t, state) [
V*cos(state(3));
V*sin(state(3));
k_k/k_s*sin(k_theta*V*t/k_s)*V
];

%% Solve and plot ODE
[t_out, state_out] = ode45(f,linspace(0,T_f,100),[0,0,0]);

scatter(state_out(:,1),state_out(:,2),'.');
daspect([1 1 1])
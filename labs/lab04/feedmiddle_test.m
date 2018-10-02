%% Setup workspace
clear;
close all
robot = raspbot('hamilton');

intialized = false;

%% Set initial variables
i_e_l = robot.encoders.LatestMessage.Vector.X;
i_e_r = robot.encoders.LatestMessage.Vector.Y;
dist = 0;
t = 0;

slippage = 1.01

target_dist = 1 * slippage;
threshold = 0.001;

prev_time = 0;

error = 0;
error_d = 0;
prev_error = 0;
error_i = 0;

kp = 0.3;
kd = 0.002;
ki = 0.1;

sref = 0;
uref = 0;

tdel = 0.13;
udel = 0;
upid = 0;
sdel = 0;

myPlot = plot(t, error, 'b-');

pref = [];
pact = [];
pdel = [];
pv = [];
pu = [];
pt = [];

pid = 1;

wh_offset = 0.98;

vmax = 0.25;
amax = 0.25;
tf = (target_dist + (vmax^2/amax))/vmax;


%% Move robot
while abs(target_dist-dist)>threshold && t<tf+1
    if ~intialized
       myc = tic();
       intialized = true;
       prev_time = toc(myc);
    end
    
    t = toc(myc);
    l_e = robot.encoders.LatestMessage.Vector.X-i_e_l;
    r_e = robot.encoders.LatestMessage.Vector.Y-i_e_r;
    dist = ((l_e+r_e)/2);
    
    dt = t - prev_time;
    
    uref = trapezoidalVelocityProfile(t, vmax, amax, target_dist, 1);
    sref = sref + uref*dt;
    
    udel = trapezoidalVelocityProfile(t-tdel, vmax, amax, target_dist, 1);
    sdel = sdel+udel*dt;
    
    error = sdel-dist;
    error_d = (error-prev_error)/dt;
    error_i = error_i + error*dt;
    
    %disp(error)
    
    upid = kp*error + kd*error_d + ki*error_i;
    
    
    V = uref + pid*upid;
    
    if abs(V)>0.3
        robot.sendVelocity(sign(V)*0.3,sign(V)*0.3*wh_offset);
    elseif abs(V)<0.02
        robot.sendVelocity(sign(V)*0.02,sign(V)*0.02*wh_offset);
    else
        robot.sendVelocity(V,V*wh_offset);
    end
    prev_time = t;
    prev_error = error;
    
    pref = [pref sref];
    pact = [pact dist];
    pdel = [pdel sdel];
    pv = [pv error];
    pt = [pt t];
end
robot.stop();


%% Plot figures
plot(pt,pref);
hold on
plot(pt,pact);
hold on
plot(pt,pdel);
legend("pref","pact","pdel")
xlabel('time')
ylabel('displacement')

figure
plot(pt,pv);
xlabel('time')
ylabel('error')
% hold on
% plot(pt,pu);
% legend("pv","pu")
disp(mean(pref-pact))
robot.stop();
robot.shutdown();
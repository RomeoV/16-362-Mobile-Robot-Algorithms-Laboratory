clear;
close all
robot = raspbot('hamilton');

intialized = false;

i_e_l = robot.encoders.LatestMessage.Vector.X
i_e_r = robot.encoders.LatestMessage.Vector.Y
dist = 0;
t = 0;
target_dist = 1.2192;
threshold = 0.001;

prev_time = 0;

error = 0;
error_d = 0;
prev_error = 0;
error_i = 0;

kp = 0.2;
kd = 0.001;
ki = 0.1;

sref = 0;
uref = 0;

tdel = 0.13;
udel = 0;
upid = 0;
sdel = 0;

myPlot = plot(t, error, 'b-');

pref = []
pact = []
pdel = []
pv = []
pu = []
pt = []

pid = 1;

wh_offset = 0.98;

vmax = 0.25;
amax = 0.25;
tf = (target_dist + (vmax^2/amax))/vmax

while abs(target_dist-dist)>threshold && t<8
    if ~intialized
       myc = tic()
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
    
    if abs(V)<0.3
        robot.sendVelocity(V,V*wh_offset);
    else
        robot.sendVelocity(sign(V)*0.3,sign(V)*0.3*wh_offset);
    end
    prev_time = t;
    prev_error = error;
    
    pref = [pref sref];
    pact = [pact dist];
    pdel = [pdel sdel];
    pv = [pv V];
    pu = [pu uref];
    pt = [pt t];
end
robot.stop();

% while t<6
%     if ~intialized
%        myc = tic()
%        intialized = true;
%        prev_time = toc(myc);
%     end
%     t = toc(myc);
%     l_e = robot.encoders.LatestMessage.Vector.X-i_e_l;
%     r_e = robot.encoders.LatestMessage.Vector.Y-i_e_r;
%     
%     dt = t - prev_time;
%     
%     dist = ((l_e+r_e)/2);
%     error = target_dist-dist;
%     uref = trapezoidalVelocityProfile(t, 0.25, 0.25, 1, 1);
%     sref = sref + uref*dt;
%     
%     udel = trapezoidalVelocityProfile(t-tdel, 0.25, 0.25, 1, 1);
%     sdel = sdel+udel*dt;
%     
%     if abs(uref)<0.3
%         robot.sendVelocity(uref,uref);
%     else
%         robot.sendVelocity(sign(uref)*0.3,sign(uref)*0.3);
%     end
%     prev_time = t;
%     pref = [pref sref];
%     pact = [pact dist];
%     pdel = [pdel sdel];
%     pt = [pt t];
% end
plot(pt,pref);
hold on
plot(pt,pact);
hold on
plot(pt,pdel);
legend("pref","pact","pdel")
figure
plot(pt,pv);
hold on
plot(pt,pu);
legend("pv","pu")
disp(mean(pref-pact))
robot.stop();
robot.shutdown();
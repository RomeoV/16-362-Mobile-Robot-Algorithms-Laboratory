clear;
close all;

robot = raspbot('sim');
d_ref = 1;
K_p = 10.0;
K_d = 0.0;
K_i = 0.0;

d_l_initial = 0; %robot.encoders.LatestMessage.Left;
d_r_initial = 0; %robot.encoders.LatestMessage.Right;
d_avg_initial = (d_l_initial+d_r_initial)/2;

while d_ref - d_travelled(robot, d_avg_initial) > 1e-4
    error = d_ref - d_travelled(robot,d_avg_initial);
    u = K_p * error;
    if u > 0.25
        u = 0.25;
    end
    robot.sendVelocity(u,u);
    pause(.05);
end
robot.shutdown();

function d = d_travelled(robot, d_avg_initial)
    d_l = robot.encoders.LatestMessage.Vector.X;
    d_r = robot.encoders.LatestMessage.Vector.Y;
    d_avg = (d_l+d_r)/2;
    d = d_avg-d_avg_initial;
end
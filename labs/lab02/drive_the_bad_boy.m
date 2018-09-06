clear;
close all;
if ~exist('robot')
    robot = raspbot('move_x_seconds')
end

robot.startLaser()
pause(2.5);
tic()
while toc() < 20
    [object_x,object_y,object_r,theta] = find_min_distance_one_shot(robot);
    [circle_x,circle_y,circle_r] = calculate_circle(object_x,object_y);
    V = 1/3 * object_r;
    if V > 0.3
        V = 0.3
    end
    if object_r < 0.20
        V = 0;
    end
    if circle_y > 0
        kappa = 1/circle_r;
    else 
        kappa = -1/circle_r;
    end
    W = 0.09;
    omega = kappa*V;
    v_r = V + W/2*omega;
    v_l = V - W/2*omega;
    if (v_r < 0.5 && v_l < 0.5)
        robot.sendVelocity(v_l,v_r);
    else
        robot.stop()
    end
    pause(.1);
end
robot.stopLaser();
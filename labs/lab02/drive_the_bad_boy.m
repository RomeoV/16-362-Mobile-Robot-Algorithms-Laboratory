clear;
close all;
robot = raspbot('hamilton')

robot.startLaser()
pause(2.5);
tic()
while toc() < 30
    [object_x,object_y,object_r,theta] = find_min_distance_one_shot(robot);
    [circle_x,circle_y,circle_r] = calculate_circle(object_x,object_y);
    %calculate curvature
    if circle_y > 0
        kappa = 1/circle_r;
    else 
        kappa = -1/circle_r;
    end
    
    %basic set speed based on distance
    V = 1/3 * object_r;
    %limit speed
    if V > 0.3
        V = 0.3
    end
    %base turning calculations
    W = 0.09;
    omega = kappa*V;
    v_r = V + W/2*omega;
    v_l = V - W/2*omega;
    
    %if were within our half meter target range rotate towards object
    if object_r < 0.5
        V = 0;
        if theta>90
            theta= theta-360;
        end
        omega = (theta/90)*2;
        v_r = V + W/2*omega;
        v_l = V - W/2*omega;
    end
    
    %When hamilton is backing up, we doint want him to spin like crazy.
    if object_r > 0.05 && object_r < 0.45
        V = ((0.75)*object_r)-0.3375;
        v_r = V;
        v_l= V;
    end
    
    %safety measures.
    if (v_r < 0.5 && v_l < 0.5)
        robot.sendVelocity(v_l,v_r);
    else
        robot.stop()
        disp("v_r");
        disp(v_r);
        disp("v_l");
        disp(v_l);
    end
    pause(.1);
end
robot.stopLaser();
robot.shutdown();
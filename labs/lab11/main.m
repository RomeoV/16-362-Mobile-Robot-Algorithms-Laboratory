clc;
close all;

sys = mrplSystem();
sys.real_time_plotting=false;

init_timer = tic();
while toc(init_timer) < 5.0
    sys.est_robot.spin(sys.robot);
end

sys.goSomewhereInWF(pose(0.3048,0.9144,pi()/2.0));
sys.rotateRobot(pi/2 - sys.est_robot.getPose().th());

sys.goSomewhereInWF(pose(0.9144,0.3048,0.0));
sys.rotateRobot(0 - sys.est_robot.getPose().th());

sys.goSomewhereInWF(pose(0.6096,0.6096,pi()/2.0));
sys.rotateRobot(pi/2 - sys.est_robot.getPose().th());

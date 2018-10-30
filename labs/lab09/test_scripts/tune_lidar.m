% Assumes sys = mrplSystem is already set up
sys.robot.startLaser()
sail = sys.findClosestSail();
disp("Sail angle: " + rad2deg(sail(3)))
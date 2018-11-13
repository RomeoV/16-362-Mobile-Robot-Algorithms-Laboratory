% Assumes sys = mrplSystem is already set up
sys.robot.startLaser()
sail = sys.findClosestSail();
assert(~isempty(sail),"No sail was found :(")
disp("Sail angle: " + rad2deg(sail(3)))
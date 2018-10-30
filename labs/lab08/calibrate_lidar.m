clear; close all;
if exist('robot') ~= 1
    robot = raspbot('hamilton'); 
end
trajectory = robotTrajectory();
log_data = logger();
log_data.logging = true;
real_time_plotting = false;

robot.forksDown();
pause(1);

current_time = 2;

% Start laser

robot.startLaser()
pause(2.5);

sails = find_sails(robot, true);
sails = sails(1:3,:);
sails(3,:) = deg2rad(sails(3,:));

disp(sails)
scatter(-sails(2,:),sails(1,:))
xlim([-1 1])
ylim([-1 1])
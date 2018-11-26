clc;
sys = mrplSystem();
sys.real_time_plotting=false;
close all;
pause(1);

x_data = [];
y_data = [];
enc_l = sys.robot.encoders.LatestMessage.Vector.X;
enc_r = sys.robot.encoders.LatestMessage.Vector.Y;
state_estimator = StateEstimator(pose(0.3,0.3,0),[enc_l;enc_r],true);

figure(11)
drive = robotKeypressDriver(gcf);

run_timer_id = tic();
while toc(run_timer_id) < 120
    drive.drive(sys.robot,1);
    enc_l = sys.robot.encoders.LatestMessage.Vector.X;
    enc_r = sys.robot.encoders.LatestMessage.Vector.Y;
    state_estimator.spin(sys.robot);
    state_estimator.updateOdometry(...
      sys.getLatestRobotTime(),...
      [enc_l, enc_r]);
    pose_est = state_estimator.getPose();

    figure(11)
    x_data = [x_data pose_est.x()];
    y_data = [y_data pose_est.y()];
    scatter(x_data,y_data,'.r'); hold on;
   quiver(pose_est.x(), pose_est.y(),...
    -cos(pose_est.th()),-sin(pose_est.th()),0.05,'filled','-.xb','LineWidth',2);
    plot([0 0 1],[1 0 0],'--g'); hold off;
    daspect([1,1,1]);
    xlim([-.5,1.5])
    ylim([-.5,1.5])
    drawnow();
    pause(0.005);
end

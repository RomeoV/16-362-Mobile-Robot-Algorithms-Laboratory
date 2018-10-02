%close all;
subplot(1,3,1)
plot(x_est_data,y_est_data);
hold on
plot(x_data,y_data,'--');
xlabel('x [m]')
ylabel('y [m]')
%legend('Estimated trajectory', 'Target trajectory')
title('Trajectory data')

subplot(1,3,2)
t_ = t_data;
plot(t_+robotModel.tdelay,trajectory.x_at_time(t_),'--')
hold on
plot(t_,x_est_data)
plot(t_+robotModel.tdelay,trajectory.y_at_time(t_),'--')
plot(t_,y_est_data)
plot(t_+robotModel.tdelay,trajectory.theta_at_time(t_),'--')
plot(t_,theta_est_data)
xlabel('time [s]')
%legend('x_{ref}','x','y_{ref}','y','theta_{ref}','theta')
title('Trajectory comparison')

subplot(1,3,3)
plot(t_data,error_x_data)
hold on
plot(t_data,error_y_data)
plot(t_data,error_theta_data)
xlabel('time [s]')
ylabel('error')
%legend('Error in x', 'Error in y', 'Erorr in theta')
title('Errors')
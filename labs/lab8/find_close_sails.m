function [cx,cy] = find_close_sails(robot)

offset = 0;

% world = scatter(lidar_x,lidar_y);
% xlim([-1 1])
% ylim([-1 1]

  r_values = circshift(robot.laser.LatestMessage.Ranges,offset);
  %Remove bad values
  th = linspace(1,360,360)';
  close_pallet = th<45 | th > 315;
  th = th(close_pallet);
  r_values = r_values(close_pallet);
  goodones = r_values>0.01 & r_values<0.3;
  r_values = r_values(goodones);
  th = th(goodones);
  
  x = cos(th).* r_values;
  y = sin(th).*r_values;
            
  center_x = mean(x);
  center_y = mean(y);
  
  cx = center_x;
  cy = center_y;
end
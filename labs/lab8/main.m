clear;
close all;
clc;
robot = raspbot('hamilton')

robot.startLaser()
pause(2.5);

logging = false;
offset = -5;

lidar_x = [];
lidar_y = [];

world = scatter(lidar_x,lidar_y);
xlim([-1 1])
ylim([-1 1])

sails = [];

tic()
firstIter = true;
while size(sails,2) < 3
  if firstIter
      tic();
      firstIter = false;
  end
  %plot points around it
  
  %Get r_values
  r_values = circshift(robot.laser.LatestMessage.Ranges,offset);
  %Remove bad values
  goodones = r_values>0.08 & r_values<1;
  r_values = r_values(goodones);
  th = linspace(1,360,360)';
  th = th(goodones);
  
  
  %Find Sail
  for i = 1:size(r_values,1)
        if r_values(i)>0
            arc_length = r_values(i)*deg2rad(1);
            pm_sail = floor(0.065/arc_length);
            pm_th = rad2deg(0.065/r_values(i));
            indices = get_ca_ij(r_values,th,th(i)-pm_th,th(i)+pm_th);
            sail_points = r_values(indices);
            theta = deg2rad(th(indices));
            x = cos(theta).* sail_points;
            y = sin(theta).*sail_points;
            
            center_x = mean(x);
            center_y = mean(y);
            
            x = x - center_x;
            y = y - center_y;
            
            %Inertia
            
            Ixx = x' * x;
            Iyy = y' * y;
            Ixy = -x' * y;
            Inertia = [Ixx Ixy;Ixy Iyy] /(size(th,1));
            lambda = eig(Inertia); 
            lambda = real(sqrt(lambda)*1000.0);
            if ~isempty(lambda) && min(lambda)<1.3 && min(lambda)>0
                sail_th = rad2deg(atan2(2*Ixy,Iyy-Ixx)/2);
                %disp(center_x + " : " + center_y + " : " + sail_th + " : " + r_values(i))
                
                %Check if near other points\
                if size(sails,2)>0
                    new_point = true;
                    for p = 1:size(sails,2)
                        sx = sails(1,p);
                        sy = sails(2,p);
                        sth = sails(3,p);
                        n = sails(4,p);
                        dist = sqrt((sx - center_x)^2 +(sy-center_y)^2);
                        if dist<0.13
                            new_point = false; 
                            sails(1,p) = ((sx*n)+center_x)/(n+1);
                            sails(2,p) = ((sy*n)+center_y)/(n+1);
                            sails(3,p) = ((sth*n)+sail_th)/(n+1);
                            sails(4,p) = n+1;
                        end
                        
                    end
                    if new_point
                        sails = horzcat(sails,[center_x;center_y;sail_th;1]); 
                    end
                else
                    sails = horzcat(sails,[center_x;center_y;sail_th;1]); 
                end
                hold on
%                 world.XData = -center_y;
%                 world.YData = center_x;
                world.XData =-sails(2,:);
                world.YData = sails(1,:);
                drawnow;
                
            end
        end
  end
  pause(0.05)
end
%disp(sails(1:3,:))
robot.stopLaser();
robot.shutdown();
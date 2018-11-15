classdef robotModel < handle
  %robotModel A convenience class for storing robot physical 
  % and performing related kinematic transforms. You can reference the 
  % defined constants via the class name with robotModel.Ws for 
  % example because they are constant properties and therefore associated
  % with the class rather than any instance. Similarly, the kinematics
  % routines are referenced from the class name as well.

  properties(Constant)
    %Robot 2
    W = 8.5/100; % wheel tread in m
    W2 = 8.5/2/100; %1/2 wheel tread in m
    maxWheelVelocity = 0.2; % max of either wheel in m/sec

    rad = .165; % robot body radius id 12.75/2 inches
    frontOffset = 6*0.0254; % front surface is 6 in in fwd of axle center
    objOffset = 1.5*0.0254; % half of object width
    laser_l = -4; % laser offset
    laser_rad = 0.04; % laser housing radius
    
    laserOffset = -7;

    tdelay = 0.23; % comms delay (bidirectional)
    
  end

  properties(Access = private)

  end

  properties(Access = public)
  end

  methods(Static = true)
      
      

    function [V, omega] = vlvrToVw(vl, vr)
      % Converts wheel speeds to body linear and angular velocity
      V = (vr+vl)/2.0;
      omega = (vr-vl)/robotModel.W;
    end

    function [vl, vr]  = VwTovlvr(V, omega)
      % Converts body linear and angular velocity to wheel speeds
      vr = V+robotModel.W2*omega;
      vl = V-robotModel.W2*omega;
    end

    function senToWorld = senToWorld(robPose)
      % Finds the sensor pose in world given the robot pose in the world.
      senToRob = pose(robotModel.laser_l,0,0);
      senToWorld = robPose.bToA()*senToRob.bToA();
    end

    function robToWorld = robToWorld(senPose)
      % Finds the robot pose in world given the sensor pose in the world.
      senToRob = pose(robotModel.laser_l,0,0);
      robToWorld = senPose.bToA()*senToRob.aToB();
    end

    function [vl, vr] = limitWheelVelocities(ctrVec)
      % Limits the speed of both wheels
      vl = ctrVec(1);
      vr = ctrVec(2);
      scale = abs(vr) / robotModel.maxWheelVelocity;

      if (scale > 1.0)
        vr = vr/scale;
        vl = vl/scale;
      end
      scale = abs(vl) / robotModel.maxWheelVelocity;
      if (scale > 1.0)
        vr = vr/scale;
        vl = vl/scale;
      end
    end

    function bodyPts = bodyGraph()
      % returns an array of pointers that can be used to plot the robot
      % body in a window

      % angle arrays
            step = pi/20;
			q1 = 0:step:pi/2;
			q2 = pi/2:step:pi;
			cir = 0:step:2*pi;          
            
            % circle for laser
			lx = robotModel.laser_rad*-cos(cir) + robotModel.laser_l;
			ly = robotModel.laser_rad*sin(cir);
			
            % body rear
			bx = [-sin(q1)*robotModel.rad lx [-sin(q2) 1  1  0]*robotModel.rad];
			by = [-cos(q1)*robotModel.rad ly [-cos(q2) 1 -1 -1]*robotModel.rad];
            
            %create homogeneous points
            bodyPts = [bx ; by ; ones(1,size(bx,2))];
        end
        
    end

    methods(Access = private)

    end

    methods(Access = public)


    end
end
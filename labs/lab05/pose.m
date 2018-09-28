classdef pose < handle
  %pose a layer over a column vector that provides access methods and
  % associated homogeneous transforms. For the purpose of naming the
  % homogeneous transforms, the pose is considered to be that of frame b
  % relative to frame a.

  properties(Constant)

  end

  properties(Access = private)
    
  end

  properties(Access = public)
    poseVec
  end

  methods(Static = true)
    function vec = matToPoseVec(mat)
      % Convert a homogeneous transform into a vector that can be
      % passed to the constructor for this class.
      x = mat(1,3);
      y = mat(2,3);
      w = atan2(-mat(1,2),mat(1,1));
      vec = [x; y; w];
    end
  end

  methods(Access = private)

  end

  methods(Access = public)
    function obj = pose(x,y,th)
      if (nargin == 1)
        sz = size(x);
        if (sz(1) == 1 || sz(2) == 1)
          obj.poseVec = x;
        elseif(sz(1) == 3 && sz(2) == 3)
          obj.poseVec = pose.matToPoseVec(x);
        end
      elseif(nargin == 3)
        obj.poseVec = [x; y; th];
      else
        obj.poseVec = [0; 0; 0];
      end
    end

    function x = x(obj); x = obj.poseVec(1); end
    function y = y(obj); y = obj.poseVec(2); end
    function th = th(obj); th = obj.poseVec(3); end

    function p = getPoseVec(obj); p = obj.poseVec; end

    function mat = bToA(obj)
      % Returns the homogeneous transform that converts coordinates from
      % the b frame to the a frame.

      mat = zeros(3,3);
      x = obj.poseVec(1); y = obj.poseVec(2); th = obj.poseVec(3);

      % ISN'T THIS WRONG, BECAUSE THE THIRD COORDINATE IS THETA, NOT THE
      % SCALING VALUE?!
      mat = [cos(th) -sin(th) x;
             sin(th)  cos(th) y;
             0.0      0.0   1.0];
    end

    function mat = aToB(obj)
      % Returns the homogeneous transform that converts coordinates from
      % the a frame to the b frame.

      bTa = bToA(obj);
      mat = inv(bTa);
    end

    function mat = bToARot(obj)
      % Returns the rotation matrix that converts coordinates from
      % the b frame to the a frame.

      mat = zeros(2,2);
      th = obj.poseVec(3);

      mat = [cos(th) -sin(th);
             sin(th)  cos(th)];
    end

    function mat = aToBRot(obj)
      % Returns the rotation matrix that converts coordinates from
      % the a frame to the b frame

      bTa = bToARot(obj);
      mat = inv(bTa);
    end
  end
end

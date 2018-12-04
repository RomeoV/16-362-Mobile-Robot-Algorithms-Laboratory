classdef rangeImage < handle
  %rangeImage Stores a 1D range image and provides related services.
  %   List of functions:
  %   Public:
  %   - rangeImage(raw_ranges)
  %   - findSailsAndWalls(obj)
  %   - getSailWidth(obj,lower_idx,upper_idx)

  %   - plotRvsTh(obj)
  %   - plotXvsY(obj)
  %   - plotSails(obj,sails)
  %   - plotWalls(obj,walls)

  %   Private:
  %   - removeBadPoints(obj)
  %   - findSailCandidate(obj,starting_idx)
  %   - calculateInertia(obj,indices)

  %   Static:
  %   - getIndices(lower_idx,upper_idx)
  %   - indicesLength(lower_idx,upper_idx)
  %   - inc(in)
  %   - dec(in)
  %   - indexAdd(a,b)
  


  properties (Constant)
    maxUsefulRange = 1.0;
    minUsefulRange = 0.06;
    maxRangeForTarget = 1.0;
  end

  properties (Access = public)
    range_vals = [];
    theta_vals = [];
    x_vals = [];
    y_vals = [];
    good_indices;
    numPix;
  end

  methods (Access = public)
    function obj = rangeImage(raw_ranges)
      % Constructs a rangeImage for the supplied data.
      % Converts the data to rectangular coordinates
        if size(raw_ranges,1) < size(raw_ranges,2)
            raw_ranges = raw_ranges';
        end
        obj.range_vals = circshift(raw_ranges,robotModel.laserOffset);
        obj.theta_vals = (1:360)';
        obj.x_vals = obj.range_vals.*cos(deg2rad(obj.theta_vals));
        obj.y_vals = obj.range_vals.*sin(deg2rad(obj.theta_vals));
        obj.numPix = max(size(raw_ranges));
        obj.removeBadPoints;
    end

    function [sails, walls] = findSailsAndWalls(obj)
        good_indices_copy = obj.good_indices;
        sails = [];
        walls = [];
        last_idx = 0;
        good_ones = find(good_indices_copy(last_idx+1:end));
        % loop through point clouds
        while ~isempty(good_ones)
            next_idx = last_idx+good_ones(1);
            % skip individual 'good points'
            indices = rangeImage.getIndices(next_idx,rangeImage.indexAdd(next_idx,2));
            if sum(obj.good_indices(indices))~=3
                good_indices_copy(next_idx:rangeImage.indexAdd(next_idx,3)) = false;
                last_idx = next_idx+3;
                good_ones = find(good_indices_copy(last_idx+1:end));
                continue
            end
            % find sail candidate for current point cloud
            [pose, lower_idx, upper_idx] = obj.findSailCandidate(next_idx+1);
            sail_angle = atan2(pose(2),pose(1)); % angle of sail center to robot
            % if sail shows in the wrong direction flip the sail angle
            if cos(pose(3))*cos(sail_angle)+sin(pose(3))*sin(sail_angle) < 0.5 
                if pose(3) < pi; pose(3) = pose(3)+pi;
                else; pose(3) = pose(3)-pi; end
            end
            
            % Determine if sail or wall based on width
            sail_width = obj.getSailWidth(lower_idx,upper_idx)/cos(pose(3)-sail_angle);
            if sail_width > 0.12 && sail_width < 0.16
                sails = [sails pose'];
            elseif sail_width > 0.16
                walls = [walls pose'];
            end
            last_idx = upper_idx;
            good_indices_copy(rangeImage.getIndices(lower_idx,upper_idx)) = false;
            
            % find next point cloud
            good_ones = find(good_indices_copy(last_idx+1:end));
        end
        
    end

    function width = getSailWidth(obj,lower_idx,upper_idx)
      avg_range = mean(obj.range_vals(rangeImage.getIndices(lower_idx,upper_idx)),'omitnan');
      sail_degrees = rangeImage.indicesLength(lower_idx,upper_idx);
      width = avg_range*deg2rad(sail_degrees);
    end


    function plotRvsTh(obj)
      % plot the range image after removing all points exceeding
      % maxRange
      figure(5)
      subplot(1,2,1)
      hold off
      polar(deg2rad(obj.theta_vals),obj.range_vals)
      hold on

    end

    function plotXvsY(obj)
      figure(5)
      subplot(1,2,2)
      hold off
      scatter(obj.x_vals,obj.y_vals,'.'); hold on;
      quiver(0,0,1,0,0.05,'filled','-.>b','LineWidth',2);      daspect([1 1 1]);
      xlim([-1,1]);
      ylim([-1,1]);
      hold on
    end
    
    function plotSails(obj,sails)
      if ~isempty(sails)
          figure(5)
          subplot(1,2,2)
          hold on

          quiver(sails(1,:), sails(2,:),...
            -cos(sails(3,:)),-sin(sails(3,:)),0.05,'filled','-.xr','LineWidth',2);      daspect([1 1 1]);
      end
    end
    
    function plotWalls(obj,walls)
      if ~isempty(walls)
          figure(5)
          subplot(1,2,2)
          hold on
          quiver(walls(1,:), walls(2,:),...
            -cos(walls(3,:)),-sin(walls(3,:)),0.05,'filled','-.xg','LineWidth',2);      daspect([1 1 1]);
      end
    end

end

  methods (Access = private)
    function removeBadPoints(obj)
      % takes all points above and below two range thresholds 
      % out of the arrays. This is a convenience but the result  
      % should not be used by any routine that expects the points 
      % to be equally separated in angle. The operation is done 
      % inline and removed data is deleted.
      obj.good_indices = obj.range_vals>obj.minUsefulRange & obj.range_vals<obj.maxUsefulRange;
      obj.range_vals(~obj.good_indices) = nan;
      obj.theta_vals(~obj.good_indices) = nan;
      obj.x_vals(~obj.good_indices) = nan;
      obj.y_vals(~obj.good_indices) = nan;
      obj.numPix = sum(obj.good_indices);
    end

    function [pose, lambda] = calculateInertia(obj,indices)
      %CALCULATEINERTIA Calculates pose and eigenvalues
      % small eigenvalue means small elipoid in that direction
      sail_points = obj.range_vals(indices);
      theta = deg2rad(obj.theta_vals(indices));
      x = cos(theta).* sail_points;
      y = sin(theta).*sail_points;

      center_x = mean(x,'omitnan');
      center_y = mean(y,'omitnan');

      x = x - center_x; x(isnan(x)) = 0;
      y = y - center_y; y(isnan(y)) = 0;

      %Inertia
      Ixx = x' * x;
      Iyy = y' * y;
      Ixy = -x' * y;
      Inertia = [Ixx Ixy;Ixy Iyy] /(size(x,1));
      lambda = eig(Inertia); 
      lambda = real(sqrt(lambda)*1000.0);

      sail_th = atan2(2*Ixy,Iyy-Ixx)/2;
      pose = [center_x,center_y,sail_th];

    end
    

    function [pose, lower_idx, upper_idx] = findSailCandidate(obj,starting_idx)
      % Find the longest sequence of pixels starting at pixel
      % “starting_idx�? until the minimum eigenvalue of the intertia
      % jumps (by 1), by adding datapoints to the left and to the right.
      % Single NaN values can be successfully skipped.
      assert(obj.good_indices(starting_idx),'Starting index not a good range value');
      
      lower_idx  = rangeImage.dec(starting_idx);
      upper_idx = rangeImage.inc(starting_idx);
      if ~obj.good_indices(lower_idx); lower_idx=rangeImage.inc(lower_idx); end
      if ~obj.good_indices(upper_idx); upper_idx=rangeImage.inc(upper_idx); end
      assert(lower_idx~=upper_idx, 'Starting point seems to have to good range neighbors')

      % expand indices to the left until eigenvalue jump
      [~, eig_vals] = obj.calculateInertia(rangeImage.getIndices(lower_idx,upper_idx));
      extended_eig_vals = eig_vals;
      while min(extended_eig_vals) - min(eig_vals) < 1
        lower_idx = rangeImage.dec(lower_idx);
        if ~obj.good_indices(lower_idx) % try to skip single 'nans'
            if ~obj.good_indices(rangeImage.dec(lower_idx)); break; 
            else; lower_idx=rangeImage.dec(lower_idx); end
        end
        eig_vals = extended_eig_vals;
        [~, extended_eig_vals] = obj.calculateInertia(rangeImage.getIndices(lower_idx,upper_idx));
      end
      lower_idx = rangeImage.inc(lower_idx);
      % catch special case with nan between sail and wall
      if ~obj.good_indices(lower_idx); lower_idx=rangeImage.inc(lower_idx); end

      % expand indices to the right until eigenvalue jump
      [~, eig_vals] = obj.calculateInertia(rangeImage.getIndices(lower_idx,upper_idx));
      extended_eig_vals = eig_vals;
      while min(extended_eig_vals) - min(eig_vals) < 1
        upper_idx = rangeImage.inc(upper_idx);
        if ~obj.good_indices(upper_idx) % try to skip single 'nans'
            if ~obj.good_indices(rangeImage.inc(upper_idx)); break; 
            else; upper_idx=rangeImage.inc(upper_idx); end
        end
        eig_vals = extended_eig_vals;
        [~, extended_eig_vals] = obj.calculateInertia(rangeImage.getIndices(lower_idx,upper_idx));
      end
      upper_idx = rangeImage.dec(upper_idx);
      % catch special case with nan between sail and wall
      if ~obj.good_indices(upper_idx); upper_idx=rangeImage.dec(upper_idx); end

      [pose,~] = obj.calculateInertia(rangeImage.getIndices(lower_idx,upper_idx));

    end
  end

  methods (Static=true)
    
    % Modulo arithmetic on nonnegative integers. MATLABs choice to
    % have matrix indices start at 1 has implications for
    % calculations relating to the position of a number in the 
    % matrix. In particular, if you want an operation defined on 
    % the sequence of numbers 1 2 3 4 that wraps around, the 
    % traditional modulus operations will not wo rk correctly. 
    % The trick is to convert the index to 0 1 2 3 4, do the 
    % math, and convert back.
    function out = inc(in)
      % increment with wraparound over natural numbers            
      out = rangeImage.indexAdd(in,1);
    end

    function out = dec(in)
      % decrement with wraparound over natural numbers
      out = rangeImage.indexAdd(in,-1);
    end

    function out = indexAdd(a,b)
      % add with wraparound over natural numbers. First number 
      % “a�? is "natural" meaning it >=1. Second number is signed.
      % Convert a to 0:3 and add b (which is already 0:3).
      % Convert the result back by adding 1.
      out = mod((a-1)+b,360)+1;
    end

    function out = indicesLength(lower_idx,upper_idx)
      if lower_idx>270 && upper_idx<90
        out = 360-lower_idx + upper_idx + 1;
      else
        out = upper_idx-lower_idx + 1;
      end
    end
    
    function out = getIndices(lower_idx,upper_idx)
      if lower_idx>270 && upper_idx<90
        out = [lower_idx:360, 1:upper_idx];
      else
        out = lower_idx:upper_idx;
      end

    end

  end
end

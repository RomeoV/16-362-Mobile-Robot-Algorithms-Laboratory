classdef lineMapLocalizer < handle
%mapLocalizer A class to match a range scan against a map in
% order to find the true location of the range scan relative to
% the map.

properties(Constant)
    maxErr = 0.05; % 5 cm
    minPts = 5; % min # of points that must match
end

properties(Access = private)

end

properties(Access = public)
    lines_p1 = [];
    lines_p2 = [];
    gain = 0.3;
    errThresh = 0.01;
    gradThresh = 0.0005;
end

methods

function obj = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)
    % create a lineMapLocalizer
    obj.lines_p1 = lines_p1;
    obj.lines_p2 = lines_p2;
    obj.gain = gain;
    obj.errThresh = errThresh;
    obj.gradThresh = gradThresh;
end

function ro2 = closestSquaredDistanceToLines(obj,pi)
    % Find the squared shortest distance from pi to any line
    % segment in the supplied list of line segments.
    % pi is an array of 2d points
    
    % throw away homogenous flag
    pi = pi(1:2,:);
    r2Array = zeros(size(obj.lines_p1,2),size(pi,2));
    for i = 1:size(obj.lines_p1,2)
        [r2Array(i,:) , ~] = closestPointOnLineSegment(pi,...
        obj.lines_p1(:,i),obj.lines_p2(:,i));
    end
    close_both = ~isinf(r2Array(1,:)) & ~isinf(r2Array(2,:));
    ro2 = min(r2Array,[],1);
    % ro2 = ro2(close_both);
end

function ids = throwOutliers(obj,pose,ptsInModelFrame)
    % Find ids of outliers in a scan.
    worldPts = pose.bToA()*ptsInModelFrame;
    r2 = obj.closestSquaredDistanceToLines(worldPts);
    ids = find(r2 < obj.maxErr*obj.maxErr);
end

function avgErr2 = fitError(obj,pose,ptsInModelFrame)
    % Find the variance of perpendicular distances of
    % all points to all lines
    % transform the points
    worldPts = pose.bToA()*ptsInModelFrame;
    r2 = obj.closestSquaredDistanceToLines(worldPts);
    r2(r2 == Inf) = [];
    err2 = sum(r2);
    num = length(r2);
    if(num >= lineMapLocalizer.minPts)
        avgErr2 = err2/num;
    else
        % not enough points to make a guess
        avgErr2 = inf;
    end
end

function [err2_Plus0,J] = getJacobian(obj,poseIn,modelPts)
    % Computes the gradient of the error function
    
    err2_Plus0 = fitError(obj,poseIn,modelPts);
    %disp(err2_Plus0)

    eps = 1e-3;
    
    dx = [eps ; 0.0 ; 0.0];
    newPose_x = pose(poseIn.getPoseVec+dx);
    error_dx = fitError(obj,newPose_x,modelPts)-err2_Plus0;
    %disp(fitError(obj,newPose_x,modelPts))
    
    dy = [0.0 ; eps ; 0.0];
    newPose_y = pose(poseIn.getPoseVec+dy);
    error_dy = fitError(obj,newPose_y,modelPts)-err2_Plus0;
    %disp(fitError(obj,newPose_y,modelPts))
    
    dz = [0.0 ; 0.0 ; eps];
    newPose_z = pose(poseIn.getPoseVec+dz);
    error_dz = fitError(obj,newPose_z,modelPts)-err2_Plus0;
    %disp(fitError(obj,newPose_z,modelPts))
    
    J = [error_dx/eps; error_dy/eps; error_dz/eps];
end

function [success, outPose] = refinePose(obj,inPose,ptsInModelFrame,maxIters)
    % refine robot pose in world (inPose) based on lidar
    % registration. Terminates if maxIters iterations is
    % exceeded or if insufficient points match the lines.
    % Even if the minimum is not found, outPose will contain 
    % any changes that reduced the fit error. Pose changes that
    % increase fit error are not included and termination
    % occurs thereafter.
    % Fill me in…
%     worldPts = inPose.bToA()*ptsInModelFrame;
%     no_corner = (worldPts(1,:).^2 + worldPts(2,:).^2)>0.02;
%     ptsInModelFrame = ptsInModelFrame(:,no_corner);
    
    ids = throwOutliers(obj,inPose,ptsInModelFrame);
    ptsInModelFrame = ptsInModelFrame(:,ids);
    
    worldPts = inPose.bToA()*ptsInModelFrame;
    no_corner = (worldPts(1,:).^2 + worldPts(2,:).^2)>0.02;
    ptsInModelFrame = ptsInModelFrame(:,no_corner);
    
    [error,J] = obj.getJacobian(inPose,ptsInModelFrame);
    mag_grad = norm(J);
    iter = 0;
    success = false;
    while (error>obj.errThresh && mag_grad>obj.gradThresh && iter<maxIters)
        inPose = pose(inPose.getPoseVec - obj.gain*J);
        [error,J] = obj.getJacobian(inPose,ptsInModelFrame);
        mag_grad = norm(J);
        iter = iter+1;
%         hold on
%         scatter(iter,mag_grad)
    end
    %disp(iter + " : " + mag_grad + " : " + error)
    if(error<obj.errThresh || mag_grad<obj.gradThresh)
        success = true;
        
    end
    outPose = inPose;
end

end

end
function [cx,cy,R] = calculate_circle(x,y)
%CALCULATE_CIRCLE Summary of this function goes here
%   Detailed explanation goes here

cx = 0;
cy = x^2/(2*y) + y/2;
R = abs(x^2/(2*y) + y/2); 

end


function uref = trapezoidalVelocityProfile(t, amax, dist, sgn)
tf = sqrt(4*dist/amax);
if t < tf/2
    uref = amax*t;
elseif t > tf/2 && t < tf
    uref = amax*(tf-t);
else
    uref = 0;
end

end
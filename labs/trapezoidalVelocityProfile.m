function uref = trapezoidalVelocityProfile(t, amax, vmax, dist, sgn)
tramp = vmax/amax;
tf = (dist + (vmax^2/amax))/vmax

if t<tramp
    uref = amax*t;
elseif (tf-t)<tramp && t<tf
    uref = amax*(tf-t);
elseif t < tf-tramp && t>tramp
    uref = vmax;
else
    uref = 0;
end

end
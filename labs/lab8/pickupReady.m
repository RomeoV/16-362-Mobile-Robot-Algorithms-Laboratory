function pickupReady(distToMove, vMax, aMax)
%move the remaining distance to place forks under the sail (don't think we
%need pid for that)
%arguments should be constant

tramp = vMax/aMax;
tf = (distToMove + (vMax^2/aMax))/vMax;
firstIter = true;
while(1)
    if(firstIter)
        tic();
    end
    t = toc;
    if t<tramp
        uref = amax*t;
    elseif (tf-t)<tramp && t<tf
        uref = amax*(tf-t);
    elseif t < tf-tramp && t>tramp
        uref = vmax;
    else
        uref = 0;
    end
    robot.sendVelocity(uref, uref);
    
    if(t>=tf)
        break;
    end
    
end

robot.stop();

end


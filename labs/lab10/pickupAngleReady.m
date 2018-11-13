function  pickupAngleReady(epsilon, smallV)
%corrects angle and lower forks when at a safe distance from the sail
%epsilon is the error tolerance for the angle
%smallV is what we use to correct the small errors
%arguments should be constant

pose = readLidarForSailPose();    %use JD's function to get sailPose (don't know the name of the function)
angle = pose(3);

while(abs(angle)>epsilon)
    
    if(angle>0)
        robot.sendVelocity(smallV, -smallV);
    else
        robot.sendVelocity(-smallV, smallV);
    end
    angle = readLidarForAngle();
    
end
robot.stop();
robot.forksDown();

end


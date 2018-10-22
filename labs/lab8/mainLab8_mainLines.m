epsilon = 5; %choose high tolerance because of the lidar's precision
safeDist = 0.15;
smallV = 0.02;
distToMove = 0.13;
done = 0;

while(1)
sailPose = readLidarForSailPose();    %use JD's function to get sailPose (don't know the name of the function)
goalPose =safeDistToSail(safeDist, sailPose);
execute_trajectory();    %put goalPose arguments inside and correct syntax
pickupAngleReady(epsilon,smallV);
pickupReady(distToMove, vMax, aMax); %if x,y position isn't good, do a second execute_trajectory
robot.forksUp();
pause(2);
robot.forksDown();
done = done+1;
if(done == 3)
    break;
end
pause(15);

end

robot.stop();
robot.shutdown();
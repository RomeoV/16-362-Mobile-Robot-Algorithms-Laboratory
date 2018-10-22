function goalPose = safeDistToSail(safeDist, sailPose)
%computes a pose that is in front of the sail but at a safe distance so we
%can adjust the angle
%safeDist should be constant

goalPose(3) = sailPose(3);
goalPose(1) = (sqrt(sailPose(1)^2+sailPose(2)^2)-safeDist)/sqrt(1+sailPose(2)^2/sailPose(1)^2);
goalPose(2) = sailPose(2)/sailPose(1)*goalPose(1);

end


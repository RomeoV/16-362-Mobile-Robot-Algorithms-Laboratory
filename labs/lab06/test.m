close all
x = 0;
y = 3;
th = pi/4;
sign = 1;
Vmax = 0.2;

%cubicSpiralTrajectory.makeLookupTable(40);
cubicSpiralTrajectory.planTrajectory(x,y,th,sign);
curve.planVelocities(Vmax);
vl_array = curve.vlArray;
vr_array = curve.vrArray;
timeArray = curve.timeArray;
figure()
plot(timeArray,vl_array)
hold on
plot(timeArray,vr_array)

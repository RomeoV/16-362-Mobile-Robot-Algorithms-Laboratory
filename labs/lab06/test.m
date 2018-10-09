close all
x = -2;
y = 0;
th = pi/4;
sign = 1;
Vmax = 0.2;

%cubicSpiralTrajectory.makeLookupTable(40);
curve = cubicSpiralTrajectory.planTrajectory(x,y,th,sign);
curve.planVelocities(Vmax);
vl_array = curve.vlArray;
vr_array = curve.vrArray;
timeArray = curve.timeArray;
figure()
plot(timeArray,vl_array)
hold on
plot(timeArray,vr_array)

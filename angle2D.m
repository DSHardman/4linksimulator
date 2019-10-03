function angle = angle2D(a,calib)
%%Assuming the accelerometer to be static, returns clockwise angle from vertical to accelerometer's y axis
%2D motion in plane perpendicular to z axis

accelerations = accread(a,calib);

for i = 1:1:2
if accelerations(i) > 9.81
    accelerations(i) = 9.81;
elseif accelerations(i) < -9.81
    accelerations(i) = -9.81;
end
end

angle1 = acos(-accelerations(2)/9.81);
%Only y reading used from this acceleration since huge variation in x readings
%{
angle2 = asin(accelerations(1)/9.81);
angle = mean([angle1,angle2]);
%}
angle = angle1;



end
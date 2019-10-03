function servoin = radtoservo(rad,minangle)
%%converts desired angle in radians to servo instruction
if rad < minangle || rad > (minangle + 2*pi/3)
    fprintf('Desired angle out of range of servo\n')
end
servoin = (rad - minangle)*3/(2*pi);
end
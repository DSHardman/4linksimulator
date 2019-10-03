function[] = newangle3(servo1,servo2,servo3,finish1,finish2,finish3)
%moves three servos to new positions using 5th, 3rd, or 1st order polynomial.
p = 30;
pausetime = 0.01;

path1 = curve5(1-readPosition(servo1),finish1,p);
%path1 = curve3(1-readPosition(servo),finish,n);
%path1 = curve1(1-readPosition(servo),finish,n);
path1 = 1-path1;

path2 = curve5(readPosition(servo2),finish2,p);
%path2 = curve3(readPosition(servo),finish,n);
%path2 = curve1(readPosition(servo),finish,n);


path3 = curve5(readPosition(servo3),finish3,p);
%path3 = curve3(readPosition(servo),finish,n);
%path3 = curve1(readPosition(servo),finish,n);
for i = 1:1:p
writePosition(servo1, path1(i));
pause(pausetime);
writePosition(servo2,path2(i));
pause(pausetime);
writePosition(servo3,path3(i));
pause(pausetime);
end

function [s,sv1,sv2,sv3] = Initialise3servos

clear s sv1 sv2 sv3

global s;
global sv1;
global sv2;
global sv3;

s=arduino();

sv1 = servo(s, 'D9', 'MinPulseDuration', 900*10^-6, 'MaxPulseDuration', 2100*10^-6);
fprintf("servo 1 initialised\n");
pause(.5);
sv2 = servo(s, 'D10', 'MinPulseDuration', 900*10^-6, 'MaxPulseDuration', 2100*10^-6);
fprintf("servo 2 initialised\n");
pause(.5);
sv3 = servo(s, 'D11', 'MinPulseDuration', 900*10^-6, 'MaxPulseDuration', 2100*10^-6);
fprintf("servo 3 initialised\n");
pause(.5);

newangle3(sv1, sv2, sv3, 0.5, 0.5, 0.5);

end
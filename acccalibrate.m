function calib = acccalibrate(a)
%%Calibrate accelerometer.
%x up/down, y up/down, z up/down with key press in between.

%standard result:
%calib = [1.5453;1.2504;1.2928;0.1197;0.2468;0.2542];

global calib
calib = zeros(3,1);
voltages = zeros(6,3);
fprintf('Position x axis up & press any key to continue\n');
pause
voltages(1,:) = [readVoltage(a,'A0') readVoltage(a,'A1') readVoltage(a,'A2')];
fprintf('Position x axis down & press any key to continue\n');
pause
voltages(2,:) = [readVoltage(a,'A0') readVoltage(a,'A1') readVoltage(a,'A2')];
fprintf('Position y axis up & press any key to continue\n');
pause
voltages(3,:) = [readVoltage(a,'A0') readVoltage(a,'A1') readVoltage(a,'A2')];
fprintf('Position y axis down & press any key to continue\n');
pause
voltages(4,:) = [readVoltage(a,'A0') readVoltage(a,'A1') readVoltage(a,'A2')];
fprintf('Position z axis up & press any key to continue\n');
pause
voltages(5,:) = [readVoltage(a,'A0') readVoltage(a,'A1') readVoltage(a,'A2')];
fprintf('Position z axis down & press any key to continue\n');
pause
voltages(6,:) = [readVoltage(a,'A0') readVoltage(a,'A1') readVoltage(a,'A2')];

calib(1) = mean(voltages(:,1));
calib(2) = mean(voltages(:,2));
calib(3) = mean(voltages(:,3));

calib(4) = abs((voltages(1,1)-voltages(2,1))/2);
calib(5) = abs((voltages(3,2)-voltages(4,2))/2);
calib(6) = abs((voltages(5,3)-voltages(6,3))/2);

fprintf('Calibration Completed\n');
end
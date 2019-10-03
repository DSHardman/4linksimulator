function accelerations = accread(a,calib)
    voltages = [readVoltage(a,'A0') readVoltage(a,'A1') readVoltage(a,'A2')];
    
    xzero = calib(1);
    yzero = calib(2);
    zzero = calib(3);
    xsens = calib(4);
    ysens = calib(5);
    zsens = calib(6);
    g = 9.81;
    
    accelerations = [0;0;0];
    accelerations(1) = g*(voltages(1)-xzero)/xsens;
    accelerations(2) = g*(voltages(2)-yzero)/ysens;
    accelerations(3) = g*(voltages(3)-zzero)/zsens;
    
end
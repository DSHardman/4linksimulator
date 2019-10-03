function servostates = phimattoservo(phiMat,minangle)
%%takes phiMat produced in inversekinematicsdemo & converts states to matrix of 
%%servo values between 0 & 1
    servostates = zeros(size(phiMat,2),3);
    for i = 1:size(phiMat,2)
        servostates(i,:) = [radtoservo(phiMat(1,i),minangle) ...
            radtoservo(phiMat(2,i),minangle) radtoservo(phiMat(3,i),minangle)];
    end
end
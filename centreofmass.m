function com = centreofmass(mVec,rMat)
%%Given the component masses and arrangement, returns coordinates of centre
%%of mass

%Mass vector order: [link1;link2;link3;link4;foot1;foot2;servo1;servo2;servo3]
%rMat in order [r00 r11 r22 r33 r44 rf1 rf2]

com = [0;0];

%extract variables
totalmass = sum(mVec);
M1 = mVec(1);
M2 = mVec(2);
M3 = mVec(3);
M4 = mVec(4);
MF1 = mVec(5);
MF2 = mVec(6);
MS1 = mVec(7);
MS2 = mVec(8);
MS3 = mVec(9);

R00 = rMat(:,1);
R11 = rMat(:,2);
R22 = rMat(:,3);
R33 = rMat(:,4);
R44 = rMat(:,5);
RF1 = rMat(:,6);
RF2 = rMat(:,7);


%x coordinate
com(1) = (1/totalmass)*(MS1*R11(1)+MS2*R22(1)+MS3*R33(1)...
    +0.5*(M1*(R11(1)+R00(1))+M2*(R22(1)+R11(1))+M3*(R33(1)+R22(1))+M4*(R44(1)+R33(1)))...
    +0.5*(MF1*(RF1(1)+R00(1))+MF2*(RF2(1)+R44(1))));

%y coordinate
com(2) = (1/totalmass)*(MS1*R11(2)+MS2*R22(2)+MS3*R33(2)...
    +0.5*(M1*(R11(2)+R00(2))+M2*(R22(2)+R11(2))+M3*(R33(2)+R22(2))+M4*(R44(2)+R33(2)))...
    +0.5*(MF1*(RF1(2)+R00(2))+MF2*(RF2(2)+R44(2))));

end
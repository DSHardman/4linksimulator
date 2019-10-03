function [rMat] = rmat1calc(phiVec,lVec,pos1)
%%Calculates rMat from phiVec when foot 1 is fixed

[A10,A21,A32] = HomCoord1(phiVec,lVec); %calculate transformation matrices

r00 = [pos1(1);pos1(2);1];
r11 = [pos1(1);pos1(2)+lVec(1);1]; %position link 1
r22 = [pos1(1);lVec(1)+pos1(2);0] + A10*[0;lVec(2);1];   %position link 2
r33 = [pos1(1);lVec(1)+pos1(2);0] + A10*A21*[0;lVec(3);1];   %position link 3
r44 = [pos1(1);lVec(1)+pos1(2);0] + A10*A21*A32*[0;lVec(4);1];   %position link 4

rf1 = [pos1(1)+lVec(5);pos1(2);0];
rf2 = r44 + [(lVec(6)/lVec(4))*(r44(2)-r33(2));(lVec(6)/lVec(4))*(r33(1)-r44(1));0];

rMat = [r00 r11 r22 r33 r44 rf1 rf2];

end
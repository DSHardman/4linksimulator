function [rMat] = rmat2calc(phiVec,lVec,pos2)
%%Calculates rMat from phiVec when foot 2 is fixed

[B10,B21,B32] = HomCoord2(phiVec,lVec); %calculate transformation matrices

r00 = [pos2(1);lVec(4)+pos2(2);0] + B32*B21*B10*[0;lVec(1);1]; %position link1
r11 = [pos2(1);lVec(4)+pos2(2);0] + B32*B21*[0;lVec(2);1]; %position link 2
r22 = [pos2(1);lVec(4)+pos2(2);0] + B32*[0;lVec(3);1];   %position link 3
r33 = [pos2(1);lVec(4)+pos2(2);1];   %position link 4
r44 = [pos2(1);pos2(2);1];

rf1 = r00 + [(lVec(5)/lVec(1))*(r11(2)-r00(2));(lVec(5)/lVec(1))*(r00(1)-r11(1));1];
rf2 = [pos2(1)-lVec(6);pos2(2);1];

rMat = [r00 r11 r22 r33 r44 rf1 rf2];

end
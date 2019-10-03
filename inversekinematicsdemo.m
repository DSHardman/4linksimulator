%Inverse kinematics - end point is entered by user

%%FOOT 1 FIXED
%draw initial configuration and get target location
fixed1(phiVec,lVec,mVec,pos1,footsep)
PDes = ginput(1)';
%Run main function
phiMat = JacInv1(phiVec,lVec,mVec,pos1,footsep,PDes);
servostates = phimattoservo(phiMat,0)


%{
%%FOOT 2 FIXED
%draw initial configuration and get target location
fixed2(phiVec,lVec,mVec,pos2,footsep)
PDes = ginput(1)';
%Run main function
JacInv2(phiVec,lVec,mVec,pos2,footsep,PDes)
%}
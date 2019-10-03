function phiMat = JacInv1(PhiVec,LVec,mVec,pos1,footsep,PDes)
%Inverse kinematics with foot 1 fixed

L1 = LVec(1);
L4 = LVec(4);
j = 0;
phiMat = [];

%get initial location and calculate initial error
[A10,A21,A32] = HomCoord1(PhiVec,LVec);
rCurr = [pos1(1);L1+pos1(2);0] + A10*A21*A32*[0;L4;1];
pErr = norm(PDes-rCurr(1:2));

%set error threshold
errThresh = 0.001;

%Main body of function
while pErr > errThresh

    %calculate Jacobian with foot 1 fixed
    J1 = Jac1(PhiVec,LVec);

    %Moore-Penrose pseudoinverse used to numerically update joint angles
    dx =  pinv(J1)*(PDes(1:2)-rCurr(1:2));
    PhiVec = PhiVec + dx;
    phiMat = [phiMat PhiVec];
    
    %draw new result: three options presented
    %redraw in target position:
    %fixed1(PhiVec,LVec,mVec,pos1,footsep)
    %or redraw in linear stages:
    %linearangleupdate(PhiVec,PhiVec-dx,LVec,mVec,pos1,footsep,1)
    %or redraw in stages following 5th order polynomial function:
    updateangles(PhiVec,PhiVec-dx,LVec,mVec,pos1,footsep,1)

    %update position and error
    [A10,A21,A32] = HomCoord1(PhiVec,LVec);
    rCurr = [pos1(1);L1+pos1(2);0] + A10*A21*A32*[0;L4;1];
    pErr = norm(PDes-rCurr(1:2));

    %increment iteration counter
    j = j+1;

    if j>30 %interrupt if position not below threshold after 30 iterations
        break
    end
end
end
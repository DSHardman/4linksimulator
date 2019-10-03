function JacInv2(PhiVec,LVec,mVec,pos2,footsep,PDes)
%Inverse kinematics with foot 2 fixed

L1 = LVec(1);
L4 = LVec(4);
j = 0;

%get initial location and calculate initial error
[B10,B21,B32] = HomCoord2(PhiVec,LVec);
rCurr = [pos2(1);L4+pos2(2);0] + B32*B21*B10*[0;L1;1];
pErr = norm(PDes-rCurr(1:2));

%set error threshold
errThresh = 0.001;

%Main body of function
while pErr > errThresh

    %calculate Jacobian with foot 1 fixed
    J2 = Jac2(PhiVec,LVec);

    %Moore-Penrose pseudoinverse used to numerically update joint angles
    dx =  pinv(J2)*(PDes(1:2)-rCurr(1:2));
    PhiVec = PhiVec + dx;
    
    %draw new result: three options presented
    %redraw in target position:
    %fixed2(PhiVec,LVec,mVec,pos2,footsep)
    %or redraw in linear stages:
    %linearangleupdate(PhiVec,PhiVec-dx,LVec,mVec,pos2,footsep,2)
    %or redraw in stages following 5th order polynomial function:
    updateangles(PhiVec,PhiVec-dx,LVec,mVec,pos2,footsep,2)

    %update position and error
    [B10,B21,B32] = HomCoord2(PhiVec,LVec);
    rCurr = [pos2(1);L4+pos2(2);0] + B32*B21*B10*[0;L1;1];
    pErr = norm(PDes-rCurr(1:2));

    %increment iteration counter
    j = j+1;

    if j>30 %interrupt if position not below threshold after 30 iterations
        break
    end
end
end
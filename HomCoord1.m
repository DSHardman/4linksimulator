function [A10,A21,A32] = HomCoord1(phiVec,lVec)
%%Returns homogeneous coordinate transform matrices with foot 1 fixed
    
    Phi1 = phiVec(1);
    Phi2 = phiVec(2);
    Phi3 = phiVec(3);

    L2 = lVec(2);
    L3 = lVec(3);

    A10 = [cos(Phi1) , sin(Phi1) , 0; ...
           -sin(Phi1) ,  cos(Phi1) , 0; ...
           0          , 0         , 1 ];

    A21 = [cos(Phi2) , sin(Phi2) , 0; ...
           -sin(Phi2) ,  cos(Phi2) , L2; ...
           0          , 0         , 1];
       
    A32 = [cos(Phi3) , sin(Phi3) , 0; ...
           -sin(Phi3) ,  cos(Phi3) , L3; ...
           0          , 0         , 1];

end
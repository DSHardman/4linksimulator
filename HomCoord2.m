function [B10,B21,B32] = HomCoord2(phiVec,lVec)
%%Returns homogeneous coordinate transform matrices with foot 2 fixed
    
    Phi1 = phiVec(1);
    Phi2 = phiVec(2);
    Phi3 = phiVec(3);

    L2 = lVec(2);
    L3 = lVec(3);

    B10 = [cos(Phi1) , -sin(Phi1) , 0; ...
           sin(Phi1) ,  cos(Phi1) , L2; ...
           0          , 0         , 1 ];

    B21 = [cos(Phi2) , -sin(Phi2) , 0; ...
           sin(Phi2) ,  cos(Phi2) , L3; ...
           0          , 0         , 1];
       
    B32 = [cos(Phi3) , -sin(Phi3) , 0; ...
           sin(Phi3) ,  cos(Phi3) , 0; ...
           0          , 0         , 1];

end
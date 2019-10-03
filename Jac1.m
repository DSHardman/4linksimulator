function J1 = Jac1(phiVec,lVec)
%%Calculates Jacobian with foot 1 fixed
    
phi1 = phiVec(1);
phi2 = phiVec(2);
phi3 = phiVec(3);

l2 = lVec(2);
l3 = lVec(3);
l4 = lVec(4);

% Jacobian
 J1 = [l2*cos(phi1)+l3*cos(phi1+phi2)+l4*cos(phi1+phi2+phi3), l3*cos(phi1+phi2)+l4*cos(phi1+phi2+phi3), l4*cos(phi1+phi2+phi3); ...
      -l2*sin(phi1)-l3*sin(phi1+phi2)-l4*sin(phi1+phi2+phi3),-l3*sin(phi1+phi2)-l4*sin(phi1+phi2+phi3),-l4*sin(phi1+phi2+phi3)];
end
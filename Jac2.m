function J2 = Jac2(phiVec,lVec)
%%Calculates Jacobian with foot 2 fixed
    
phi1 = phiVec(1);
phi2 = phiVec(2);
phi3 = phiVec(3);

l1 = lVec(1);
l2 = lVec(2);
l3 = lVec(3);

% Jacobian
 J2 = [-l1*cos(phi1+phi2+phi3), -l2*cos(phi2+phi3)-l1*cos(phi1+phi2+phi3), -l3*cos(phi3)-l2*cos(phi2+phi3)-l1*cos(phi1+phi2+phi3); ...
      -l1*sin(phi1+phi2+phi3),-l2*sin(phi2+phi3)-l1*sin(phi1+phi2+phi3),-l3*sin(phi3)-l2*sin(phi2+phi3)-l1*sin(phi1+phi2+phi3)];
end
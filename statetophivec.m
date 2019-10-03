%convert from state to phiVec
function phiVec = statetophivec(state,servomin,stepsize,n)
    phiVec = [0;0;0];
    baseexp = dec2base(state-1,n,3);
    for i = 1:3
        phiVec(i) = servomin + (str2double(baseexp(i)))*stepsize;
    end
end
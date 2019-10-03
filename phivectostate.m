%convert from phiVec positions to state
function state = phivectostate(phiVec,servomin,stepsize,n)
    for i = 1:length(phiVec)
        phiVec(i) = round((phiVec(i)-servomin)/stepsize);
    end
    basenstring = strcat(num2str(phiVec(1)),num2str(phiVec(2)),num2str(phiVec(3)));
    state = 1 + base2dec(basenstring,n);
end
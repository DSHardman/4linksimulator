function servostates = pathtakentoservo(pathtaken,minangle,servomin,stepsize,n)
%%takes pathtaken produced by simulannwalk & converts states to matrix of
%%servo values between 0 & 1

servostates = [];

for i = 1:size(pathtaken,1)
    for j = 1:size(find(pathtaken(i,:)),2)
        phiVec = statetophivec(pathtaken(i,j),servomin,stepsize,n);
        servostates = [servostates; radtoservo(phiVec(1),minangle) radtoservo(phiVec(2),minangle) radtoservo(phiVec(3),minangle)];
    end
end

end
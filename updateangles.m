function updateangles(phiVecNew,phiVecOld,lVec,mVec,pos,footsep,fixed,corners)
%%Smoothly animates transition between 2 phiVecs, using 5th order
%%polynomial.

f = 5;  %set number of frames
t0 = 1;

%Calculate paths to be taken
plan1 = curve5u(phiVecOld(1),phiVecNew(1),f,t0);
plan2 = curve5u(phiVecOld(2),phiVecNew(2),f,t0);
plan3 = curve5u(phiVecOld(3),phiVecNew(3),f,t0);

%for each frame, draw new position
for i = 1:f
    phiVec = [plan1(i);plan2(i);plan3(i)];
    %Foot 1 fixed
    if fixed == 1
        %Draw obstacles if they have been entered.
        switch nargin
            case 7
                fixed1(phiVec,lVec,mVec,pos,footsep)
            case 8
                fixed1(phiVec,lVec,mVec,pos,footsep,corners)
        end
    end
    %Foot 2 fixed
    if fixed == 2
        %Draw obstacles if they have been entered
        switch nargin
            case 7
                fixed2(phiVec,lVec,mVec,pos,footsep)
            case 8 
                fixed2(phiVec,lVec,mVec,pos,footsep,corners)
        end
    end
    
    %Error message for incorrect input variable
    if fixed ~= 1 && fixed ~=2
        fprintf('fixed value invalid')
    end  
end
end
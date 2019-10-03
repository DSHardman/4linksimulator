function linearangleupdate(phiVecNew,phiVecOld,lVec,mVec,pos,footsep,fixed,corners)
%%Animates motion between 2 phiVecs by updating frames linearly

f = 5;  %set number of frames

for i = 1:f
    phiVec = phiVecOld + (i/f)*(phiVecNew-phiVecOld);
    if fixed == 1
        fixed1(phiVec,lVec,mVec,pos,footsep,corners)
    end
    if fixed == 2
        fixed2(phiVec,lVec,mVec,pos,footsep,corners)
    end
    if fixed ~= 1 && fixed ~=2
        fprintf('fixed value invalid')
    end  
end
end
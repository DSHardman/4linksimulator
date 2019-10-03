function posb = performpath(path,lVec,mVec,posa,footsep,fixed,servomin,stepsize,n,corners)
%%Animates path containing states in order visited, with one foot fixed.
%Returns pos1 or pos2 of unfixed foot.

    phiVec = statetophivec(path(1),servomin,stepsize,n);
    for i = 2:1:length(path)
        phiVecNew = statetophivec(path(i),servomin,stepsize,n);
        %Animate using either 5th order polynomial or linearly segmented
        %frames
        %updateangles(phiVecNew,phiVec,lVec,mVec,posa,footsep,fixed,corners);
        linearangleupdate(phiVecNew,phiVec,lVec,mVec,posa,footsep,fixed,corners);
        phiVec = phiVecNew;
    end
    
    %return posb value
    switch fixed
        case 1
            rMat = rmat1calc(phiVec,lVec,posa);
            posb = rMat(:,5);
        case 2
            rMat = rmat2calc(phiVec,lVec,posa);
            posb = rMat(:,1);
    end
    
end
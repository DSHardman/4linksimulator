function endstates = endstatesearch(sstatespace,ustatespace,startstate,corners,lVec,mVec,fixed,pos,servomin,stepsize,n)
%%Searches statespaces for valid end states with free foot on floor or
%%obstacle.
%sstatespace is stable, ustatespace is unstable
%Preference given to stable end states, else looks for paths to tip itself
%into end state.

endstates = [];
startphiVec = statetophivec(startstate,servomin,stepsize,n);

%first look for stable solutions
switch fixed
    %foot 1 fixed in place
    case 1
        rMatinitial = rmat1calc(startphiVec,lVec,pos);
        for j = 1:length(sstatespace)
        state = base2dec(sstatespace(j,:),n) + 1;
        phiVec = statetophivec(state,servomin,stepsize,n);
        rMat = rmat1calc(phiVec,lVec,pos);
        [flatground,gnd] = onground(rMat,2,corners);
        com = centreofmass(mVec,rMat);
        unstable = stability(com,rMat(1:2,5),gnd,lVec,2);
        if ~unstable && rMat(1,5) > rMatinitial(1,5) && flatground == 1
            distance = rMat(1,5) - rMat(1,6);
            endstates = [endstates; j 0 distance];
        end
        end
    case 2
        %foot 2 fixed in place
        rMatinitial = rmat2calc(startphiVec,lVec,pos);
        for j = 1:length(sstatespace)
        state = base2dec(sstatespace(j,:),n) + 1;
        phiVec = statetophivec(state,servomin,stepsize,n);
        rMat = rmat2calc(phiVec,lVec,pos);
        [flatground,gnd] = onground(rMat,1,corners);
        com = centreofmass(mVec,rMat);
        unstable = stability(com,pos,gnd,lVec,2);
        if flatground == 1 && ~unstable && rMat(1,1) > rMatinitial(1,1) + 0.1
            distance = rMat(1,5) - rMat(1,6);
            endstates = [endstates; j 0 distance]; 
        end
        end
end

%finish here if any stable solutions have been found
if ~isempty(endstates)
    return
end

%Search for endstates involving tipping over on the fixed foot.
for ang = pi/64:pi/64:pi/2
    switch fixed
        case 1
            for j = 1:length(ustatespace)
            state = base2dec(ustatespace(j,:),n) + 1;
            phiVec = statetophivec(state,servomin,stepsize,n);
            rMat = rmat1calc(phiVec,lVec,pos);
            com = centreofmass(mVec,rMat);
            [~,gnd] = onground(rMat,1,corners);
            unstable1 = stability(com,pos,gnd,lVec,1);
            
            rMat = rotateclockwise(rMat(1:2,6),rMat,ang);
            [flatground,gnd] = onground(rMat,2,corners);
            
            ocoll = 0;
            for i = 1:length(corners)/2
                if obstaclecollide(rMat,corners(:,2*i-1:2*i))
                    ocoll = 1;
                end
            end
            com = centreofmass(mVec,rMat);
            unstable2 = stability(com,rMat(1:2,5),gnd,lVec,2);
            
            %endstates should have no collisions, be stable on the new
            %foot, but unstable on the old.
            if flatground && ~ocoll && ~unstable2 && unstable1
                distance = rMat(1,5) - rMat(1,6);
                endstates = [endstates; j ang distance];
            end
            end
            
            
            
        case 2
            for j = 1:length(ustatespace)
            state = base2dec(ustatespace(j,:),n) + 1;
            phiVec = statetophivec(state,servomin,stepsize,n);
            rMat = rmat2calc(phiVec,lVec,pos);
            rMat = rotateclockwise(rMat(1:2,7),rMat,-ang);
            [flatground,~] = onground(rMat,1,corners);
            if flatground == 1
                distance = rMat(1,5) - rMat(1,6);
                endstates = [endstates; j ang distance]; 
            end
            end
    end
    %preference given to smallest possible angles: if any valid endstates
    %are found then the function ends here.
    if ~isempty(endstates)
        return
    end
    
end

if isempty(endstates)
    fprintf('NO VALID ENDSTATES FOUND\n')
end
end
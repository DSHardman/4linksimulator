function [statespace] =...
    createstatespace(lVec,mVec,pos,fixed,footsep,servomin,stepsize,n,corners,stab)
%%Returns a base-n representation of all valid states - no collisions with
%%obstacles, ground or self.
%stab boolean = 1: only stable states are returned.
%stab boolean = 0: both stable and unstable states are returned.

%Initialise statespace
states = 1:n^3;
statespace = [];

%Eliminate all invalid states (i.e. only add valid states)
for i = 1:n^3
    phiVec = statetophivec(i,servomin,stepsize,n);
    if fixed == 1
        rMat = rmat1calc(phiVec,lVec,pos);
        [flatground,gnd] = onground(rMat,1);
        [ground,joint] = throughground(rMat);
    elseif fixed == 2
        rMat = rmat2calc(phiVec,lVec,pos);
        [flatground,gnd] = onground(rMat,2);
        [ground,joint] = throughground(rMat);
    end
    %Self collision
    scollide = selfcollide(rMat,footsep);
    %stability
    com = centreofmass(mVec,rMat);
    unstable = stability(com,pos,gnd,lVec,fixed);
    %obstacle collisions
    ocoll = 0;
    for j = 1:length(corners)/2
       if obstaclecollide(rMat,corners(:,2*j-1:2*j))
           ocoll = 1;
       end
    end
    
    if stab == 1
        %add only valid states to statespace
        if  ground ~= 1 && ocoll ~= 1 && scollide ~= 1 && unstable ~= 1
            statespace = [statespace; dec2base(states(i)-1,n,3)];
        end
    elseif stab == 0
        %case when stability isn't considered in compiling statespace
        if ground ~= 1 && ocoll ~= 1 && scollide ~= 1
            statespace = [statespace; dec2base(states(i)-1,n,3)];
        end
    end

end

end
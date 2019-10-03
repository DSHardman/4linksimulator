function [statespace,startindex,endindex] = generalsimulatedannealsetup(startphivec,endphivec,lVec,mVec,posa,fixed,footsep,servomin,stepsize,n,corners)
    %%Prepares for simulated annealing script between start/end phivecs,
    %%avoiding any obstacles.
    
    %Returns statespace of valid states, and indices of the start/end
    %states in this space
    
    %Search for valid states in the discretised (base-n) space
    statespace = createstatespace(lVec,mVec,posa,fixed,footsep,servomin,stepsize,n,corners,1);
    
    %convert start & end phiVecs to closest state
    for i = 1:3
        startup = servomin + ceil((startphivec(i)-servomin)/stepsize)*stepsize;
        startdown = servomin + floor((startphivec(i)-servomin)/stepsize)*stepsize;
        if (startphivec(i)-startdown)/(startup-startdown) >= 0.5
            startphivec(i) = startup;
        else
            startphivec(i) = startdown;
        end

        endup = servomin + ceil((endphivec(i)-servomin)/stepsize)*stepsize;
        enddown = servomin + floor((endphivec(i)-servomin)/stepsize)*stepsize;
        if (endphivec(i)-enddown)/(endup-enddown) >= 0.5
            endphivec(i) = endup;
        else
            endphivec(i) = enddown;
        end
    end
    
    %Convert phivecs to state in base-n system
    startstate = phivectostate(startphivec,servomin,stepsize,n);
    endstate = phivectostate(endphivec,servomin,stepsize,n);
    
    %Convert base-n to index in valid space
    startindex = findindices(statespace,startstate,n);
    if startindex == 0
        error('Start phivec is invalid');
    end
    endindex = findindices(statespace,endstate,n);
    if endindex == 0
        error('End phivec is invalid');
    end
    
end
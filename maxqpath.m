function [path,servostates] = maxqpath(Q,startstate,endstates,servomin,stepsize,n,options)
%%After Q learning, returns an array of the best path found & its corresponding
%%servo commands 

    maxaction = zeros(length(Q));
    
    %maxaction contains the action with highest probability from each state
    for i = 1:length(Q)
       [~,maxaction(i)] = max(Q(:,i));
       clear unused
    end
    
    path = startstate;
    state = startstate;
    B = 0;
    j = 0;
    %Follow this maximum path until endstate or 100 iterations are reached.
    while B == 0 && j < 100
       selection = options(:,maxaction(state));
       phiVec = change3(selection(1),selection(2),statetophivec(state,servomin,stepsize,n),stepsize);
       state = phivectostate(phiVec,servomin,stepsize,n);
       B = endstates == state;
       path = [path;state];
       pause(1)
       j = j+1;
    end
    if j == 100
        fprinf('Too many steps: valid endpoint not reached')
    end
    
    servostates = pathtakentoservo(path,0,servomin,stepsize,n);
end
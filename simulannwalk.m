function [pathtaken,posVec] = simulannwalk(startstate,lVec,mVec,pos1,footsep,servomin,stepsize,n,corners,T,testspertemp,tfraction,tempnumber)
%%Uses simulated annealing to walk forwards & deal with obstacles.
%Returns all steps taken in matrix form, padded with zeros where necessary.
%PosVec contains pos 1 & 2 values at each step.
%Learning parameters are defined in simulated annealing function.

finalpath = startstate;
posVec = pos1;
pathtaken = [];

while pos1(1) < 3
    %Step Outwards
    
    %Define steady & unsteady statespaces from current position
    sstatespace = createstatespace(lVec,mVec,pos1,1,footsep,servomin,stepsize,n,corners,1);
    instatespace = 0;
    startstate = dec2base(finalpath(end)-1,n,3);
    for i = 1:size(sstatespace,1)
        if sstatespace(i,:) == startstate
            instatespace = 1;
        end
    end
    if ~instatespace
        sstatespace = [sstatespace;startstate];
    end
    
    ustatespace = createstatespace(lVec,mVec,pos1,1,footsep,servomin,stepsize,n,corners,0);
    instatespace = 0;
    for i = 1:size(ustatespace,1)
        if ustatespace(i,:) == startstate
            instatespace = 1;
        end
    end
    if ~instatespace
        ustatespace = [ustatespace;startstate];
    end
    
    %Search for valid endstates from current position
    endstates = endstatesearch(sstatespace,ustatespace,finalpath(end),corners,lVec,mVec,1,pos1,servomin,stepsize,n);
    
    if endstates(1,2) == 0 %if an endstate was found which does not require tipping
       [~,loc] = max(endstates(:,3)); %find valid state which maximises stride length
       startindex = findindices(sstatespace,finalpath(end),n);
       endindex = endstates(loc,1);
       %Find efficient & valid path between the 2
       [finalpath,~] = simulatedannealing(n,sstatespace,startindex,endindex,T,testspertemp,tfraction,tempnumber);
       %Animate path found
       performpath(finalpath,lVec,mVec,pos1,footsep,1,servomin,stepsize,n,corners);
       %Update positional values
       phiVec = statetophivec(finalpath(end),servomin,stepsize,n);
       rMat = rmat1calc(phiVec,lVec,pos1);
       pos2 = rMat(1:2,5);
       pos2 = levelfoot(pos2,corners);
       
    else %if tipping is required
       [~,loc] = max(endstates(:,3)); %find valid state which maximises stride length
       startindex = findindices(ustatespace,finalpath(end),n);
       endindex = endstates(loc,1);
       %Find efficient & valid path between the 2
       [finalpath,~] = simulatedannealing(n,ustatespace,startindex,endindex,T,testspertemp,tfraction,tempnumber);
       %Animate path found
       performpath(finalpath,lVec,mVec,pos1,footsep,1,servomin,stepsize,n,corners);
       %Update positional values
       phiVec = statetophivec(finalpath(end),servomin,stepsize,n);
       rMat = rmat1calc(phiVec,lVec,pos1);
       %Rotate matrix & update again
       rMat = rotateclockwise(rMat(1:2,6),rMat,endstates(loc,2));
       pos2 = rMat(1:2,5);
       pos2 = levelfoot(pos2,corners); %ensure foot is on top of obstacle or ground
       %draw new tipped configuration
       drawrmat(rMat,corners)
    end
   
    %add path just found to pathtaken matrix, and pad any space with zeros.
    if length(finalpath) == size(pathtaken,2)
        pathtaken = [pathtaken; finalpath];
    elseif length(finalpath) < size(pathtaken,2)
        padding = zeros(1,size(pathtaken,2)-length(finalpath));
        pathtaken = [pathtaken; [finalpath padding]];
    elseif length(finalpath) > size(pathtaken,2)
        padding = zeros(size(pathtaken,1),length(finalpath)-size(pathtaken,2));
        pathtaken = [[pathtaken padding]; finalpath];
    end
    
    %Update posVec
    posVec = [posVec pos2];
    
    %Step Inwards
    %Define steady & unsteady statespaces from current position
    sstatespace = createstatespace(lVec,mVec,pos2,2,footsep,servomin,stepsize,n,corners,1);
    instatespace = 0;
    startstate = dec2base(finalpath(end)-1,n,3);
    for i = 1:size(sstatespace,1)
        if sstatespace(i,:) == startstate
            instatespace = 1;
        end
    end
    if ~instatespace
        sstatespace = [sstatespace;startstate];
    end
    
    ustatespace = createstatespace(lVec,mVec,pos2,2,footsep,servomin,stepsize,n,corners,0);
    instatespace = 0;
    for i = 1:size(ustatespace,1)
        if ustatespace(i,:) == startstate
            instatespace = 1;
        end
    end
    if ~instatespace
        ustatespace = [ustatespace;startstate];
    end
    
    
    %Search for valid endstates from current position
    endstates = endstatesearch(sstatespace,ustatespace,finalpath(end),corners,lVec,mVec,2,pos2,servomin,stepsize,n);

   if endstates(1,2) == 0
       [~,loc] = min(endstates(:,3)); %if an endstate was found which does not require tipping
       startindex = findindices(sstatespace,finalpath(end),n); %find valid state which maximises stride length
       
       endindex = endstates(loc,1);
       %Find efficient & valid path between the 2
       
       [finalpath,~] = simulatedannealing(n,sstatespace,startindex,endindex,T,testspertemp,tfraction,tempnumber);
       %Animate path found
       performpath(finalpath,lVec,mVec,pos2,footsep,2,servomin,stepsize,n,corners);
       %Update positional values
       phiVec = statetophivec(finalpath(end),servomin,stepsize,n);
       rMat = rmat2calc(phiVec,lVec,pos2);
       pos1 = rMat(1:2,1);
       pos1 = levelfoot(pos1,corners);
       
   %avoid any solutions requiring falling backwards when aiming to move in
   %one direction
   else
       fprintf('New state requires backwards tilt\n')
       return
   end
   
   %add path just found to pathtaken matrix, and pad any space with zeros.
   if length(finalpath) == size(pathtaken,2)
       pathtaken = [pathtaken; finalpath];
    elseif length(finalpath) < size(pathtaken,2)
        padding = zeros(1,size(pathtaken,2)-length(finalpath));
       pathtaken = [pathtaken; [finalpath padding]];
    elseif length(finalpath) > size(pathtaken,2)
        padding = zeros(size(pathtaken,1),length(finalpath)-size(pathtaken,2));
       pathtaken = [[pathtaken padding]; finalpath];
   end
   
   %Update posVec
   posVec = [posVec pos1];
    
end
end
function coarsegeneralqlearn1(startphivec,endphivec,corners,n,alpha,gamma,iterationno,stepsperiteration,endreward,punish,lVec,mVec,pos1,footsep)

%Q Learning with foot 1 fixed to ground
%Desired start and finish angles are entered and a path between them is
%learned.
%Obstacles entered in corners variable.

%%PARAMETERS
%learning rates
%alpha = 0.8;
%gamma = 0.8;
%graphics update
pausetime = 0.01;
%start & end positions
%startphivec = [0;0;0];
%endphivec = [pi/2;0;pi/2];
%servo limits
servomin = 0;
servomax = pi/2;
%n = 3;
stepsize = (servomax-servomin)/(n-1);
%iterations
%iterationno = 50;
%stepsperiteration = 50;
%reward values
%endreward = 3;
%punish = -1;

successes = 0;

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

startstate = phivectostate(startphivec,servomin,stepsize,n);
endstate = phivectostate(endphivec,servomin,stepsize,n);

%initialise Q-values for possible actions in each state
Q1 = ones(6,n^3);


%set invalid choices
for i = 1:n^3
    for j = 1:3
        phiVec = statetophivec(i,servomin,stepsize,n);
        if round(phiVec(j),1) == round(servomin,1)
            Q1(2*j-1,i) = 0;
        end
        if round(phiVec(j),1) == round(servomax,1)
            Q1(2*j,i) = 0;
        end
    end
end


%setrewards
R(1:n^3) = 0;
R(endstate) = endreward;
for i = 1:length(R)
    phiVec = statetophivec(i,servomin,stepsize,n);
    
    rMat = rmat1calc(phiVec,lVec,pos1);
    [ground,~] = throughground(rMat);
    scollide = selfcollide(rMat,footsep);
    com = centreofmass(mVec,rMat);
    unstable = stability(com,pos1,0,lVec,1);
    [flatground,~] = onground(rMat,2);
    ocoll = 0;
    for j = 1:length(corners)/2
        if obstaclecollide(rMat,corners(:,2*j-1:2*j))
            ocoll = 1;
        end
    end
    
    if ground == 1 || scollide == 1 || unstable == 1 || ocoll == 1
        R(i) = punish;
    end
    
    if flatground == 1 && i ~= endstate
        R(i) = punish;
    end
end


%set options from any state [joint,direction]
options = [];
for i = 1:3
    options = [options [i;0] [i;1]];
end

for j = 1:iterationno
    
    set(gcf,'color','w');
    phiVec = startphivec;
    %draw starting configuration
    fixed1(phiVec,lVec,mVec,pos1,footsep,corners)
    newstate = startstate;
    pause(pausetime);
    %main body
    for i = 1:stepsperiteration
        oldstate = newstate;
        action = nextaction(Q1(:,newstate));
        selection = options(:,action);
        phiVec = change3(selection(1),selection(2),phiVec,stepsize);
        newstate = phivectostate(phiVec,servomin,stepsize,n);
        %update graphics
        fixed1(phiVec,lVec,mVec,pos1,footsep,corners)
        %smoother option - 5th order polynomial:
        %updateangles(phiVec,statetophivec(oldstate,servomin,stepsize,n),lVec,mVec,pos1,footsep,1,corners)
        pause(pausetime);
        %update Q-Values
        Q1(action,oldstate) = (1-alpha)*Q1(action,oldstate) + alpha*(R(newstate) + gamma*max(Q1(:,newstate)));
        if Q1(action,oldstate) < 0
            Q1(action,oldstate) = 0.01;
        end
        if newstate == endstate
            set(gcf,'color','g');
            successes = successes + 1;
            pause(1)
            break
        end
        title = ['Iteration: ' num2str(j),'   Step: ',num2str(i),'   End reached: ',num2str(successes)];
        set(get(gca, 'title'), 'string',title)
    end
    fprintf('iteration %d\n',j)
end


%function chooses next action based on state array

function [action] = nextaction(p)
    action = randsample(1:6,1,true,p);
end


%{
%softmax (base e) to choose next action
function[action] = nextaction(p)
s = p;
for i2 = 1:length(p)
    %keep invalid options as impossible
    if p(i2) ~= 0
        s(i2) = exp(p(i2));
    end
end
action = randsample(1:6,1,true,s);
%note exponential denominator is not required since randsample normalises the array
end
%}

%{
%function chooses action with max probability or randomises between those of equal value
function [action] = nextaction(p)
maxval = max(p);
idx = find(p == maxval);
if length(idx) > 1
    action = randsample(idx,1);
else
    action = idx;
end
end
%}

end
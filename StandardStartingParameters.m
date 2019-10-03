%%Initialises useful starting parameters.

%Original Testing parameters:

phiVec = [pi/3;pi/3;pi/3];
lVec = [1.2;1.4;1.4;1.2;1.5;1.5];
mVec = [1;1;1;1;1;1;1;1;1];
pos1 = [-2;0]; %Starting position of joint between link 1 & foot 1
pos2 = [0;0]; %Starting position of joint between link 4 & foot 2
footsep = 0.5; %Minimum distance feet can be apart on ground without colliding

%Physical Limits of servo:
servomin = 0;
servomax = pi/2;

%Q-Learning/Simulated annealing discretises each servo into n possible states
n = 3;
stepsize = (servomax-servomin)/(n-1);

%% ADD OBSTACLES
%corners = [[2.6;0] [4.5;0.2] [-5;5.8] [5;6]]; %DEFAULT

%corners uses [[bottomleft1] [topright1] [bottomleft2] [topright2]...] format
corners = [[2.6;0] [4.5;0.2] [-5;5.8] [5;6]];


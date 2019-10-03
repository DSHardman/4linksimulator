%Arduino & sensors must first have been initialised
%Moves sv3 only, using accelerometer to intermediately calculate the
%orientation of link 1

%% Parameters
N=30; % number of samples
dt = 1;
t=0:dt:dt*N; %total time
wt = pi/(3*N*dt); % true angular velocity
pausetime = 1; %before readings


Xinitial = 0; %initial angle
%set starting position
newangle3(sv1,sv2,sv3,0,0,0);
%create true path
Xt = Xinitial + wt * t;

figure;
hold on;
axis([0 N 0 2]);

% Estimation

% state of the robot is X= [pos, vel]. 
%initial guess: robot starts at pos = 0 with velocity equals to 50% of the real velocity
Xk_prev = [0; .5*wt];

% Current state estimate
Xk=[];

% system dynamics matrix 
A = [1 dt; 0  1];

% Xk = A*Xk_prev + Noise, that is Xk(n) = Xk(n-1) + Vk(n-1) * dt
% V is estimated 

% Error matrix P 
sigma_model = 1; % we are giving same weight to new measurement and to the model estimate 
P = [sigma_model, 0;0, sigma_model];

% process noise covariance.
Q = [0.1, 0;0, 0.1];


% Measurement matrix. 
C = [1 0]; % we are able to measure the position, but not the velocity


% Measurement noise covariance. Generally R and sigma_meas can vary between samples. 
sigma_meas = 5; 
R = sigma_meas^2;


% Kalman filter

% Buffers declaration
Xk_buffer = zeros(2,N+1);
Xk_buffer(:,1) = Xk_prev;
Z_buffer = zeros(1,N+1);

for k=1:N
    
    writePosition(sv1, 1-Xt(k)*3/(2*pi));
    pause(pausetime);
    
    Z = angle2D(s,calib);
    Z_buffer(k+1) = Z;
    

    % Kalman iteration
    P1 = A*P*A' + Q;
    S = C*P1*C' + R;
   
    K = P1*C'*inv(S); % kalman gain 
    P = P1 - K*C*P1;
    
    Xk = A*Xk_prev + K*(Z-C*A*Xk_prev);
    Xk_buffer(:,k+1) = Xk;
    
    Xk_prev = Xk; 
    
%Plot current iteration
plot(t(1:k),Xt(1:k),'k'); %groundtruth
scatter(t(1:k),Z_buffer(1:k),'r','x'); % measurements
plot(t(1:k),Xk_buffer(1,1:k),'g'); % estimate
title('Kalman Filter Position Estimation');
xlabel('Step');
ylabel('Angle [Rad]');
legend('Model position','Measurements','Kalman estimation');
end


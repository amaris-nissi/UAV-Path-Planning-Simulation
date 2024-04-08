close all; clear; clc;
rng('shuffle');

Position = (readmatrix("a_star_plyn_pos.csv"));
Velocity = (readmatrix("a_star_plyn_vel.csv"));
Accelera = (readmatrix("a_star_plyn_acc.csv"));

t_vec = Position(1,1:end)';
r_vec = Position(2:3,1:end)';
v_vec = Velocity(2:3,1:end)';
a_vec = Accelera(2:3,1:end)';

N = length(t_vec);

R.tVec = t_vec;
R.rIstar = [r_vec(:,1),r_vec(:,2),zeros(N,1)];
R.vIstar = [v_vec(:,1),v_vec(:,2),zeros(N,1)];
R.aIstar = [a_vec(:,1),a_vec(:,2),zeros(N,1)];
R.xIstar = diag(1./vecnorm(R.vIstar'))*(-R.vIstar); %[ones(N,1),zeros(N,2)];

S.distMat = 0*randn(N-1,3) ;% Matrix of disturbance forces acting on the body
S.state0.r = [0 0 1]';% Initial position in m
S.state0.e = [0 0 pi]';% Initial attitude expressed as Euler angles, in radians
S.state0.v = [0 0 0]';% Initial velocity of body with respect to I, expressed in I, in m/s
S.state0.omegaB = [0 0 0]';% Initial angular rate of body with respect to I, expressed in B, in rad/s
S.oversampFact = 2;% Oversampling factor
S.rXIMat = [0,0,0; 0,0,0]; % Feature locations in the I frame

quadParamsScript;
constantsScript;
sensorParamsScript;
P.quadParams = quadParams; 
P.constants = constants; 
P.sensorParams = sensorParams;

Q = simulateQuadrotorEstimationAndControl(R,S,P);

figure(5);clf;
plot(Q.state.rMat(:,1), Q.state.rMat(:,2)); 
axis equal; grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Horizontal position of CM');
function [Xdot] = quadOdeFunctionHF(t,X,eaVec,distVec,P)
% quadOdeFunctionHF : Ordinary differential equation function that models
%                     quadrotor dynamics -- high-fidelity version.  For use
%                     with one of Matlab's ODE solvers (e.g., ode45).
%
%
% INPUTS
%
% t ---------- Scalar time input, as required by Matlab's ODE function
%              format.
%
% X ---------- Nx-by-1 quad state, arranged as 
%
%              X = [rI',vI',RBI(1,1),RBI(2,1),...,RBI(2,3),RBI(3,3),...
%                   omegaB',omegaVec']'
%
%              rI = 3x1 position vector in I in meters
%              vI = 3x1 velocity vector wrt I and in I, in meters/sec
%             RBI = 3x3 attitude matrix from I to B frame
%          omegaB = 3x1 angular rate vector of body wrt I, expressed in B
%                   in rad/sec
%        omegaVec = 4x1 vector of rotor angular rates, in rad/sec.
%                   omegaVec(i) is the angular rate of the ith rotor.
%
%    eaVec --- 4x1 vector of voltages applied to motors, in volts.  eaVec(i)
%              is the constant voltage setpoint for the ith rotor.
%
%  distVec --- 3x1 vector of constant disturbance forces acting on the quad's
%              center of mass, expressed in Newtons in I.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
% OUTPUTS
%
% Xdot ------- Nx-by-1 time derivative of the input vector X
%
%+------------------------------------------------------------------------------+

% Initialization
zB = [ 0 ; 0 ; 1] ; % z-axis of the body reference frame B has it's origin at the quadrotor center of mass - CM
time = t; % Scalar time input, as required by Matlab's ODE function format
rI = [X(1);X(2);X(3)] ; % 3x1 position vector in I in meters
vI = [X(4);X(5);X(6)] ; % 3x1 velocity vector wrt I and in I, in meters/sec
RBI = [X(7),X(10),X(13);X(8),X(11),X(14);X(9),X(12),X(15)]; % 3x3 attitude matrix from I to B frame
omegaB = [X(16);X(17);X(18)]; % 3x1 angular rate vector of body wrt I, expressed in B in rad/sec
omegaVec = [X(19);X(20);X(21);X(22)]; % 4x1 vector of rotor angular rates, in rad/sec

% Calculate Forces and Torques vectors from given scalar values
Fi1 = P.quadParams.kF(1) * (eaVec(1) * P.quadParams.cm(1))^2 ; % Scalar forces
Fi2 = P.quadParams.kF(2) * (eaVec(2) * P.quadParams.cm(2))^2 ; % Scalar forces
Fi3 = P.quadParams.kF(3) * (eaVec(3) * P.quadParams.cm(3))^2 ; % Scalar forces
Fi4 = P.quadParams.kF(4) * (eaVec(4) * P.quadParams.cm(4))^2 ; % Scalar forces
% Calculate Forces and Torques vectors for each rotor
F1 = Fi1 .* zB ; % Bold F: external force due to 1st rotor
F2 = Fi2 .* zB ; % Bold F: external force due to 2nd rotor
F3 = Fi3 .* zB ; % Bold F: external force due to 3rd rotor
F4 = Fi4 .* zB ; % Bold F: external force due to 4th rotor
% sum of Forces vectors 
Fisum = F1 + F2 + F3 + F4 ; % sum of F (not bold)
% Defining torques of body frame
N1 = [ 0 ; 0 ; - P.quadParams.kN(1) * P.quadParams.omegaRdir(1) * (eaVec(1) * P.quadParams.cm(1))^2 ] ; % Vector Torques
N2 = [ 0 ; 0 ; - P.quadParams.kN(2) * P.quadParams.omegaRdir(2) * (eaVec(2) * P.quadParams.cm(2))^2 ] ; % Vector Torques
N3 = [ 0 ; 0 ; - P.quadParams.kN(3) * P.quadParams.omegaRdir(3) * (eaVec(3) * P.quadParams.cm(3))^2 ] ; % Vector Torques
N4 = [ 0 ; 0 ; - P.quadParams.kN(4) * P.quadParams.omegaRdir(4) * (eaVec(4) * P.quadParams.cm(4))^2 ] ; % Vector Torques
NB1 = N1 + cross( P.quadParams.rotor_loc(:,1) , F1 ) ; % Equ(3.7)
NB2 = N2 + cross( P.quadParams.rotor_loc(:,2) , F2 ) ; % Equ(3.7)
NB3 = N3 + cross( P.quadParams.rotor_loc(:,3) , F3 ) ; % Equ(3.7)
NB4 = N4 + cross( P.quadParams.rotor_loc(:,4) , F4 ) ; % Equ(3.7)
NB = NB1 + NB2 + NB3 + NB4 ;

% Approximation to the aerodynamic forces acting on the quad
zI = rI .* zB;
fd =  norm(vI)^2 * dot(vI,zI) / (norm(zI) * norm(vI) + 1e-7) ;
vIu = vI/(norm(vI)+1e-7) ;
da = 1/2 * P.quadParams.Cd * P.quadParams.Ad * P.constants.rho * fd ; % 1/2 Cd Ad ρ fd(zI,vI)
drag_force = - da * vIu ;

% Derivative of each component of X big
rIDot = vI ; % Equ(3.5)
vIDot =  drag_force + ([0 ; 0 ; -P.constants.g*P.quadParams.m] + (transpose(RBI) * Fisum) + distVec)/P.quadParams.m ; % Equ(3.6)
RbiDot = (-crossProductEquivalent(omegaB)) * RBI ; % Equ(3.8)
omegaBDot = (inv(P.quadParams.Jq)) * (NB - (crossProductEquivalent(omegaB) * P.quadParams.Jq * omegaB)) ; % Equ(3.7)
omegaVec_dot_1 = ( 1 / P.quadParams.taum(1)) * (eaVec(1) * P.quadParams.cm(1) - omegaVec(1)) ;
omegaVec_dot_2 = ( 1 / P.quadParams.taum(2)) * (eaVec(2) * P.quadParams.cm(2) - omegaVec(2)) ;
omegaVec_dot_3 = ( 1 / P.quadParams.taum(3)) * (eaVec(3) * P.quadParams.cm(3) - omegaVec(3)) ;
omegaVec_dot_4 = ( 1 / P.quadParams.taum(4)) * (eaVec(4) * P.quadParams.cm(4) - omegaVec(4)) ;

% Assembling X big dot to work with Ode45()
%Xdot = [ rIDot ; vIDot ; RbiDot(:) ; omegaBDot ; omegaVec_dot ] ;
Xdot = [rIDot(1);rIDot(2);rIDot(3);...
    vIDot(1);vIDot(2);vIDot(3);...
    RbiDot(1,1);RbiDot(2,1);RbiDot(3,1);...
    RbiDot(1,2);RbiDot(2,2);RbiDot(3,2);...
    RbiDot(1,3);RbiDot(2,3);RbiDot(3,3);...
    omegaBDot(1);omegaBDot(2);omegaBDot(3);...
    omegaVec_dot_1;omegaVec_dot_2;omegaVec_dot_3;omegaVec_dot_4...
    ];

end
function [ftildeB,omegaBtilde] = imuSimulator(S,P)
% imuSimulator : Simulates IMU measurements of specific force and
%                body-referenced angular rate.
%
% INPUTS
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position of CM in the I frame, in meters
%
%                  vI = 3x1 velocity of CM with respect to the I frame and
%                       expressed in the I frame, in meters per second.
%
%                  aI = 3x1 acceleration of CM with respect to the I frame and
%                       expressed in the I frame, in meters per second^2.
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude of B frame wrt I frame
%
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
%           omegaBdot = 3x1 time derivative of omegaB, in radians per
%                       second^2.
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the
%                 quad's sensors, as defined in sensorParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
% OUTPUTS
%
% ftildeB ---- 3x1 specific force measured by the IMU's 3-axis accelerometer
%
% omegaBtilde  3x1 angular rate measured by the IMU's 3-axis rate gyro
%
%+------------------------------------------------------------------------------+

% Maintaine the values of ba and bg
persistent ba bg
if(isempty(ba))
% Set ba’s initial value
QbaSteadyState = P.sensorParams.Qa2/(1 - P.sensorParams.alphaa^2);
ba = mvnrnd(zeros(3,1), QbaSteadyState)';
end
if(isempty(bg))
% Set bg’s initial value
QbgSteadyState = P.sensorParams.Qg2/(1 - P.sensorParams.alphag^2);
bg = mvnrnd(zeros(3,1), QbgSteadyState)';
end

% Inputs
aI = S.statek.aI ;
RBI = S.statek.RBI ; 
omegaB = S.statek.omegaB ;
Qa = P.sensorParams.Qa ;
Qa2 = P.sensorParams.Qa2 ;
Qg = P.sensorParams.Qg ;
Qg2 = P.sensorParams.Qg2 ;
gravity = P.constants.g ;

% Initialize
e3 = [0;0;1] ;
va  = mvnrnd(zeros(3,1), Qa   )';
va2 = mvnrnd(zeros(3,1), Qa2  )';
vg  = mvnrnd(zeros(3,1), Qg   )';
vg2 = mvnrnd(zeros(3,1), Qg2  )';

% Compute
ba_tk1 = P.sensorParams.alphaa * ba + va2 ;
ba = ba_tk1 ;
bg_tk1 = P.sensorParams.alphag * bg + vg2 ;
bg = bg_tk1 ;

% Ouputs
ftildeB = RBI*(aI+(gravity*e3)) + ba + va ;
omegaBtilde = omegaB + bg + vg ;

end
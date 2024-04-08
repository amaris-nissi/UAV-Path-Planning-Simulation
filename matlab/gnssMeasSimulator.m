function [rpGtilde,rbGtilde] = gnssMeasSimulator(S,P)
% gnssMeasSimulator : Simulates GNSS measurements for quad.
%
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
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude of B frame wrt I frame
%
% P ---------- Structure with the following elements:
%
%  sensorParams = Structure containing all relevant parameters for the
%                 quad's sensors, as defined in sensorParamsScript.m 
%
%
% OUTPUTS
%
% rpGtilde --- 3x1 GNSS-measured position of the quad's primary GNSS antenna,
%              in ECEF coordinates relative to the reference antenna, in
%              meters.
%
% rbGtilde --- 3x1 GNSS-measured position of secondary GNSS antenna, in ECEF
%              coordinates relative to the primary antenna, in meters.
%              rbGtilde is constrained to satisfy norm(rbGtilde) = b, where b
%              is the known baseline distance between the two antennas.
%
%+------------------------------------------------------------------------------+ 

% Inputs
rI = S.statek.rI ; % 3x1 position of CM in the I frame, in meters
RBI = S.statek.RBI ; % 3x3 direction cosine matrix of B frame wrt I frame
rA = P.sensorParams.raB ; % = [0.1013 -0.0887; -0.0004 -0.0004; 0.0472 0.0472];
rpB = rA(:,1) ; % Primary GNSS antenna in the body frame
rsB = rA(:,2) ; % Secondary GNSS antenna in the body frame
RpL = P.sensorParams.RpL ; % = diag([0.006^2, 0.006^2, 0.012^2]);
sigmab = P.sensorParams.sigmab ; % = 0.05; 
r0G = P.sensorParams.r0G ; % [-742080.456; -5462030.875; 3198338.956];

% Initialize
I = eye(3);
Epsilon = 10^(-8) ;

% Compute
% Primary Antenna Position Measurement Model
rpI = rI + transpose(RBI) * rpB ;
RLG = Recef2enu(r0G) ;
RIG = RLG ;
rpG = transpose(RIG) * rpI ;
RpG = (inv(RLG) * RpL) * inv(transpose(RLG));
Rp_wpG = RpG * I ;
wpG = mvnrnd(zeros(3,1), Rp_wpG)' ;

% Baseline Vector Measurement Model
rsI = rI + transpose(RBI) * rsB ;
rsG = transpose(RIG) * rsI ;
rbG = rsG - rpG ;
rubG = rbG / norm(rbG) ;
RbG = norm(rbG)^2 * sigmab^2 * (I - rubG * transpose(rubG)) + (Epsilon * I) ;
Rb_wpG = RbG * I ;
wbG = mvnrnd(zeros(3,1), Rb_wpG)' ;

% Outputs
rpGtilde = rpG + wpG ;
rbGtilde = rbG + wbG ;

end
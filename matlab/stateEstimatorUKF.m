function [E] = stateEstimatorUKF(S,M,P)
% stateEstimatorUKF : Unscented-Kalman-filter-based estimation of the state of
%                     quadcopter from sensor measurements.  
%
% INPUTS
%
% S ---------- Structure with the following elements:
%
%        rXIMat = Nf-by-3 matrix of coordinates of visual features in the
%                 simulation environment, expressed in meters in the I
%                 frame. rXIMat(i,:)' is the 3x1 vector of coordinates of
%                 the ith feature.  
%
%          delt = Measurement update interval, in seconds.
%
% M ---------- Structure with the following elements:
%
%            tk = Time at which all measurements apply, in seconds.
%
%      rpGtilde = 3x1 GNSS-measured position of the quad's primary GNSS
%                 antenna, in ECEF coordinates relative to the reference
%                 antenna, in meters.
%
%      rbGtilde = 3x1 GNSS-measured position of secondary GNSS antenna, in
%                 ECEF coordinates relative to the primary antenna, in meters.
%                 rbGtilde is constrained to satisfy norm(rbGtilde) = b, where b
%                 is the known baseline distance between the two antennas.
%
%         rxMat = Nf-by-2 matrix of measured positions of feature point
%                 projections on the camera's image plane, in
%                 pixels. rxMat(i,:)' is the 2x1 image position measurement of
%                 the 3D feature in rXIMat(i,:)'.  If the ith feature point is
%                 not visible to the camera (the ray from the feature to the
%                 camera center never intersects the image plane), then
%                 rxMat(i,:) = [NaN, NaN].               
%
%       ftildeB = 3x1 specific force measured by the IMU's 3-axis
%                 accelerometer, in meters/sec^2
%
%   omegaBtilde = 3x1 angular rate measured by the IMU's 3-axis rate gyro,
%                 in rad/sec. 
%
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
%  sensorParams = Structure containing sensor parameters, as defined in
%                 sensorParamsScript.m
%
%
% OUTPUTS
%
% E ---------- Structure with the following elements:
%
%        statek = Estimated state of the quad at tk, expressed as a structure
%                 with the following elements. 
%                   
%                  rI = 3x1 position of CM in the I frame, in meters
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude
%
%                  vI = 3x1 velocity of CM with respect to the I frame and
%                       expressed in the I frame, in meters per second.
%                 
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
%                  ba = 3x1 bias of accelerometer expressed in the
%                       accelerometer frame in meter/second^2.
%
%                  bg = 3x1 bias of rate gyro expressed in the body frame in
%                       rad/sec.
%
%            Pk = nx-by-nx error covariance matrix for the estimator state
%                 vector xk that applies at time tk, which is defined as 
% 
%                 xk = [rI', vI', e', ba', bg']'
%
%                 where all corresponding quantities are identical to those
%                 defined for E.statek and where e is the 3x1 error Euler
%                 angle vector defined such that for an estimate RBIHat of the
%                 attitude, the true attitude is RBI = dRBI(e)*RBIHat, where
%                 dRBI(e) is the DCM formed from the error Euler angle vector
%                 e, expressed in radians.
%
%+------------------------------------------------------------------------------+

%----- Setup
persistent xBark PBark RBIBark
RIG = Recef2enu(P.sensorParams.r0G);
rbB = P.sensorParams.raB(:,2) - P.sensorParams.raB(:,1); rbBu = rbB/norm(rbB);
rpB = P.sensorParams.raB(:,1);  e3 = [0;0;1];
epsilon = 1e-8; nx = 15; nv = 12;
alphaUKF = 1e-3; betaUKF = 2; kappaUKF = 0; 
lambda_p = alphaUKF^2*(kappaUKF + nx + nv) - nx - nv;
c_p = sqrt(nx+nv+lambda_p);
Wm0_p = lambda_p/(nx + nv + lambda_p);
Wmi_p = 1/(2*(nx + nv + lambda_p));
Wc0_p = Wm0_p + 1 - alphaUKF^2 + betaUKF; Wci_p = Wmi_p;

%----- Convert GNSS measurements to I frame
rpItilde = RIG*M.rpGtilde;
rbItilde = RIG*M.rbGtilde; rbItildeu = rbItilde/norm(rbItilde);

%----- Initialize state estimate on first call to this function
if(isempty(xBark))
  vIMat = [rbItildeu'; e3']; vBMat = [rbBu'; e3']; aVec = ones(2,1);
  RBIBark = wahbaSolver(aVec,vIMat,vBMat);
  rIBark = rpItilde - RBIBark'*rpB;
  xBark = [rIBark; zeros(12,1)];
  QbaSteadyState = P.sensorParams.Qa2/(1 - P.sensorParams.alphaa^2);
  QbgSteadyState = P.sensorParams.Qg2/(1 - P.sensorParams.alphag^2);
  PBark = diag([2*diag(P.sensorParams.RpL); 
                0.001*ones(3,1); ...
                2*P.sensorParams.sigmab^2*ones(3,1); ...
                diag(QbaSteadyState); diag(QbgSteadyState)]);  
end

%----- Assemble measurements
zk = [rpItilde; rbItildeu];
RcCCellArray = {}; jj = 1; Nfk = 0;
[Nf,~] = size(M.rxMat);
mcVeck = zeros(Nf,1); 
for ii=1:Nf
  if(~isnan(M.rxMat(ii,1)))
    mcVeck(ii) = 1;
    viCtilde = [P.sensorParams.pixelSize*M.rxMat(ii,:)';P.sensorParams.f];
    norm_viCtilde = norm(viCtilde);
    viCtildeu = viCtilde/norm_viCtilde; 
    % viCtildeu is the measured unit vector pointing from the camera (C) frame
    % origin to the 3D feature point, expressed in C.
    zk = [zk; viCtildeu];
    % Error covariance matrix for the unit vector viCtildeu    
    sigmac = sqrt(P.sensorParams.Rc(1,1))*P.sensorParams.pixelSize/norm_viCtilde;
    RcC = sigmac^2*(eye(3) - viCtildeu*viCtildeu') + epsilon*eye(3);
    RcCCellArray{jj} = RcC;
    jj = jj + 1; 
    Nfk = Nfk + 1;    
  end
end

%----- Perform measurement update
% Form measurement error covariance matrix Rk
RpI = P.sensorParams.RpL;
rbIu = RBIBark'*rbBu;
RbI = P.sensorParams.sigmab^2*(eye(3)-rbIu*rbIu') + epsilon*eye(3);
Rk = blkdiag(RpI,RbI);
for ii=1:Nfk
  Rk = blkdiag(Rk,RcCCellArray{ii});
end
nz = length(zk);
lambda_u = alphaUKF^2*(kappaUKF + nx + nz) - nx - nz;
c_u = sqrt(nx+nz+lambda_u);
Wm0_u = lambda_u/(nx + nz + lambda_u);
Wmi_u = 1/(2*(nx + nz + lambda_u));
Wc0_u = Wm0_u + 1 - alphaUKF^2 + betaUKF;
Wci_u = Wmi_u;
% Form augmented a priori state and error covariance matrix
xBarAugk = [xBark; zeros(nz,1)];
PBarAugk = blkdiag(PBark,Rk);
SxBar = chol(PBarAugk)';
% Assemble sigma points and push these through the measurement function
sp0 = xBarAugk;
spMat = zeros(nx+nz, 2*(nx+nz));  zpMat = zeros(nz,2*(nx+nz));
zp0 = h_meas(sp0(1:nx),sp0(nx+1:end),RBIBark,S.rXIMat,mcVeck,P);
for ii=1:2*(nx+nz)
  jj = ii; pm = 1;
  if(ii > (nx + nz)) jj = ii - nx - nz; pm = -1; end
  spMat(:,ii) = sp0 + pm*c_u*SxBar(:,jj);
  zpMat(:,ii) = h_meas(spMat(1:nx,ii),spMat(nx+1:end,ii),RBIBark,S.rXIMat,mcVeck,P);
end
% Recombine sigma points
zBark = sum([Wm0_u*zp0, Wmi_u*zpMat],2);
Pzz = Wc0_u*(zp0 - zBark)*(zp0 - zBark)';
Pxz = Wc0_u*(sp0(1:nx) - xBark)*(zp0 - zBark)';
for ii=1:2*(nx+nz)
  Pzz = Pzz + Wci_u*(zpMat(:,ii) - zBark)*(zpMat(:,ii) - zBark)';
  Pxz = Pxz + Wci_u*(spMat(1:nx,ii) - xBark)*(zpMat(:,ii) - zBark)';
end
% Perform LMMSE measurement update
PzzInv = inv(Pzz);
xHatk = xBark + Pxz*PzzInv*(zk - zBark);
Pk = PBark - Pxz*PzzInv*Pxz';

%----- Package output state
statek.rI = xHatk(1:3); statek.vI = xHatk(4:6); ek = xHatk(7:9);
statek.RBI = euler2dcm(ek)*RBIBark;
statek.ba = xHatk(10:12); statek.bg = xHatk(13:15);
statek.omegaB = M.omegaBtilde - statek.bg; 
PkDiag = diag(Pk);
E.statek = statek;
E.Pk = Pk;
 
if(0)
% Testing section
tk = M.tk
[statek.rI - statekTrue.rI]
[statek.vI - statekTrue.vI]
dcm2euler(statekTrue.RBI' * statek.RBI)*180/pi
sqrt(PkDiag(7:9))*180/pi
pause;
clc;
end

%----- Propagate state to time tkp1
RBIHatk = statek.RBI; xHatk(7:9) = zeros(3,1);
Qk = blkdiag(P.sensorParams.Qg,P.sensorParams.Qg2,...
  P.sensorParams.Qa,P.sensorParams.Qa2);
xHatAugk = [xHatk; zeros(nv,1)];
PAugk = blkdiag(Pk,Qk);
Sx = chol(PAugk)';
% Assemble sigma points and push these through the dynamics function
sp0 = xHatAugk;
xpMat = zeros(nx,2*(nx+nv));
uk = [M.omegaBtilde;M.ftildeB];
xp0 = f_dynamics(sp0(1:nx),uk,sp0(nx+1:end),S.delt,RBIHatk,P);
for ii=1:2*(nx+nv)
  jj = ii; pm = 1;
  if(ii > (nx + nv)) jj = ii - nx - nv; pm = -1; end
  spii = sp0 + pm*c_p*Sx(:,jj);
  xpMat(:,ii) = f_dynamics(spii(1:nx),uk,spii(nx+1:end),S.delt,RBIHatk,P);
end
% Recombine sigma points
xBarkp1 = sum([Wm0_p*xp0, Wmi_p*xpMat],2);
PBarkp1 = Wc0_p*(xp0 - xBarkp1)*(xp0 - xBarkp1)';
for ii=1:2*(nx+nv)
  PBarkp1 = PBarkp1 + ...
            Wci_p*(xpMat(:,ii) - xBarkp1)*(xpMat(:,ii) - xBarkp1)';
end
ekp1 = xBarkp1(7:9);
RBIBarkp1 = euler2dcm(ekp1)*RBIHatk;
xBarkp1(7:9) = zeros(3,1);
% Set k = kp1 in preparation for next iteration
RBIBark = RBIBarkp1; xBark = xBarkp1; PBark = PBarkp1;







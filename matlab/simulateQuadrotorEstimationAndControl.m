function [Q] = simulateQuadrotorEstimationAndControl(R,S,P)
% simulateQuadrotorEstimationAndControl : Simulates closed-loop estimation and
%                                         control of a quadrotor aircraft.
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%          tVec = Nx1 vector of uniformly-sampled time offsets from the
%                 initial time, in seconds, with tVec(1) = 0.
%
%        rIstar = Nx3 matrix of desired CM positions in the I frame, in
%                 meters.  rIstar(k,:)' is the 3x1 position at time tk =
%                 tVec(k).
%
%        vIstar = Nx3 matrix of desired CM velocities with respect to the I
%                 frame and expressed in the I frame, in meters/sec.
%                 vIstar(k,:)' is the 3x1 velocity at time tk = tVec(k).
%
%        aIstar = Nx3 matrix of desired CM accelerations with respect to the I
%                 frame and expressed in the I frame, in meters/sec^2.
%                 aIstar(k,:)' is the 3x1 acceleration at time tk =
%                 tVec(k).
%
%        xIstar = Nx3 matrix of desired body x-axis direction, expressed as a
%                 unit vector in the I frame. xIstar(k,:)' is the 3x1
%                 direction at time tk = tVec(k).
%  
% S ---------- Structure with the following elements:
%
%  oversampFact = Oversampling factor. Let dtIn = R.tVec(2) - R.tVec(1). Then
%                 the output sample interval will be dtOut =
%                 dtIn/oversampFact. Must satisfy oversampFact >= 1.
%
%        state0 = State of the quad at R.tVec(1) = 0, expressed as a structure
%                 with the following elements:
%                   
%                   r = 3x1 position in the world frame, in meters
% 
%                   e = 3x1 vector of Euler angles, in radians, indicating the
%                       attitude
%
%                   v = 3x1 velocity with respect to the world frame and
%                       expressed in the world frame, in meters per second.
%                 
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
%       distMat = (N-1)x3 matrix of disturbance forces acting on the quad's
%                 center of mass, expressed in Newtons in the I frame.
%                 distMat(k,:)' is the constant (zero-order-hold) 3x1
%                 disturbance vector acting on the quad from R.tVec(k) to
%                 R.tVec(k+1).
%
%        rXIMat = Nf-by-3 matrix of coordinates of visual features in the
%                 simulation environment, expressed in meters in the I
%                 frame. rXIMat(i,:)' is the 3x1 vector of coordinates of
%                 the ith feature.  
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
% Q ---------- Structure with the following elements:
%
%          tVec = Mx1 vector of output sample time points, in seconds, where
%                 Q.tVec(1) = R.tVec(1), Q.tVec(M) = R.tVec(N), and M =
%                 (N-1)*oversampFact + 1.
%  
%         state = State of the quad at times in tVec, expressed as a
%                 structure with the following elements:
%                   
%                rMat = Mx3 matrix composed such that rMat(k,:)' is the 3x1
%                       position at tVec(k) in the I frame, in meters.
% 
%                eMat = Mx3 matrix composed such that eMat(k,:)' is the 3x1
%                       vector of Euler angles at tVec(k), in radians,
%                       indicating the attitude.
%
%                vMat = Mx3 matrix composed such that vMat(k,:)' is the 3x1
%                       velocity at tVec(k) with respect to the I frame
%                       and expressed in the I frame, in meters per
%                       second.
%                 
%           omegaBMat = Mx3 matrix composed such that omegaBMat(k,:)' is the
%                       3x1 angular rate vector expressed in the body frame in
%                       radians, that applies at tVec(k).
%
%+------------------------------------------------------------------------------+

N = length(R.tVec);
dtIn = R.tVec(2) - R.tVec(1);
dtOut = dtIn/S.oversampFact;
RBIk = euler2dcm(S.state0.e);
omegaVec0 = zeros(4,1);
Xk = [S.state0.r;S.state0.v;RBIk(:);S.state0.omegaB;omegaVec0];
Xdotk = zeros(length(Xk),1);
statek.RBI = zeros(3,3);
[Nf,~] = size(S.rXIMat);
Se.rXIMat = S.rXIMat;
Se.delt = dtIn;
XMat = []; tVec = [];

for kk=1:N-1
  % Simulate measurements
  statek.rI = Xk(1:3);
  statek.RBI(:) = Xk(7:15);
  statek.vI = Xk(4:6);
  statek.omegaB = Xk(16:18);
  statek.aI = Xdotk(4:6);
  statek.omegaBdot = Xdotk(16:18);
  Sm.statek = statek;
  % Simulate measurements
  M.tk=dtIn*(kk-1);
  [M.rpGtilde,M.rbGtilde] = gnssMeasSimulator(Sm,P);
  M.rxMat = [];
  for ii=1:Nf
    rx = hdCameraSimulator(S.rXIMat(ii,:)',Sm,P);
    if(isempty(rx))
      M.rxMat(ii,:) = [NaN,NaN];
    else
      M.rxMat(ii,:) = rx';
    end
  end
  [M.ftildeB,M.omegaBtilde] = imuSimulator(Sm,P);
  % Call estimator
  E = stateEstimatorUKF(Se,M,P); 
  if(~isempty(E.statek))
    % Call trajectory and attitude controllers
    Rtc.rIstark = R.rIstar(kk,:)';
    Rtc.vIstark = R.vIstar(kk,:)';
    Rtc.aIstark = R.aIstar(kk,:)';
    Rac.xIstark = R.xIstar(kk,:)';
    distVeck = S.distMat(kk,:)';
    Sc.statek = E.statek;
    [Fk,Rac.zIstark] = trajectoryController(Rtc,Sc,P);
    NBk = attitudeController(Rac,Sc,P);
    % Convert commanded Fk and NBk to commanded voltages
    eaVeck = voltageConverter(Fk,NBk,P);
  else
    % Apply no control if state estimator's output is empty.  Set distVeck to
    % apply a normal force in the vertical direction that exactly offsets the
    % acceleration due to gravity.
    eaVeck = zeros(4,1);
    distVeck = [0;0;P.quadParams.m*P.constants.g];
  end
  tspan = [R.tVec(kk):dtOut:R.tVec(kk+1)]';
  [tVeck,XMatk] = ...
      ode45(@(t,X) quadOdeFunctionHF(t,X,eaVeck,distVeck,P),tspan,Xk);
  if(length(tspan) == 2)
    % Deal with S.oversampFact = 1 case 
    tVec = [tVec; tVeck(1)];
    XMat = [XMat; XMatk(1,:)];
  else
    tVec = [tVec; tVeck(1:end-1)];
    XMat = [XMat; XMatk(1:end-1,:)];
  end
  Xk = XMatk(end,:)';
  Xdotk = quadOdeFunctionHF(tVeck(end),Xk,eaVeck,distVeck,P);
  % Ensure that RBI remains orthogonal
  if(mod(kk,100) == 0)
   RBIk(:) = Xk(7:15);
   [UR,~,VR]=svd(RBIk);
   RBIk = UR*VR'; Xk(7:15) = RBIk(:);
  end
end
XMat = [XMat;XMatk(end,:)];
tVec = [tVec;tVeck(end,:)];

M = length(tVec);
Q.tVec = tVec;
Q.state.rMat = XMat(:,1:3);
Q.state.vMat = XMat(:,4:6);
Q.state.omegaBMat = XMat(:,16:18);
Q.state.eMat = zeros(M,3);
RBI = zeros(3,3);
for mm=1:M
  RBI(:) = XMat(mm,7:15);
  Q.state.eMat(mm,:) = dcm2euler(RBI)';  
end


  


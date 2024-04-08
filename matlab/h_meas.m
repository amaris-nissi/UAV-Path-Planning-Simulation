function [zk] = h_meas(xk,wk,RBIBark,rXIMat,mcVeck,P)
% h_meas : Measurement model for quadcopter.
%
% INPUTS
%
% xk --------- 15x1 state vector at time tk, defined as 
% 
%              xk = [rI', vI', e', ba', bg']'
%
%              where all corresponding quantities are identical to those
%              defined for E.statek in stateEstimatorUKF.m and where e is the
%              3x1 error Euler angle vector defined such that for an estimate
%              RBIBar of the attitude, the true attitude is RBI =
%              dRBI(e)*RBIBar, where dRBI(e) is the DCM formed from the error
%              Euler angle vector e.
%
% wk --------- nz-by-1 measurement noise vector at time tk, defined as
%
%              wk = [wpIk', wbIk', w1C', w2C', ..., wNfkC']'
%
%              where nz = 6 + Nfk*3, with Nfk being the number of features
%              measured by the camera at time tk, and where all 3x1 noise
%              vectors represent additive noise on the corresponding
%              measurements.
%
% RBIBark ---- 3x3 attitude matrix estimate at time tk.
%
% rXIMat ----- Nf-by-3 matrix of coordinates of visual features in the
%              simulation environment, expressed in meters in the I
%              frame. rXIMat(i,:)' is the 3x1 vector of coordinates of the ith
%              feature.
%
% mcVeck ----- Nf-by-1 vector indicating whether the corresponding feature in
%              rXIMat is sensed by the camera: If mcVeck(i) is true (nonzero),
%              then a measurement of the visual feature with coordinates
%              rXIMat(i,:)' is assumed to be made by the camera.  mcVeck
%              should have Nfk nonzero values.
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
% zk --------- nz-by-1 measurement vector at time tk, defined as
%
%              zk = [rpItilde', rbItildeu', v1Ctildeu', ..., vNfkCtildeu']'
%
%              where rpItilde is the 3x1 measured position of the primary
%              antenna in the I frame, rbItildeu is the 3x1 measured unit
%              vector pointing from the primary to the secondary antenna,
%              expressed in the I frame, and viCtildeu is the 3x1 unit vector,
%              expressed in the camera frame, pointing toward the ith 3D
%              feature, which has coordinates rXIMat(i,:)'.
%
%+------------------------------------------------------------------------------+

% Inputs
rI = xk(1:3) ;
vI = xk(4:6) ;
e = xk(7:9) ;
ba = xk(10:12) ;
bg = xk(13:15) ;
wpIk = wk(1:3) ;
wbIk = wk(4:6) ;
wC_i = wk(7:length(wk)) ;

% Initialize
wk_i = [];
rpB = P.sensorParams.raB(:,1);
rsB = P.sensorParams.raB(:,2);
rcB = P.sensorParams.rocB;
RCB = P.sensorParams.RCB;
Nfk = length(wC_i) ;
Nf = length(mcVeck);

% Compute
dRBI = euler2dcm(e);
RBI = dRBI * RBIBark ;
rpI = (rI + transpose(RBI) * rpB);
rsI = (rI + transpose(RBI) * rsB);
rbI = rsI - rpI ;
rbI_u = rbI ./ (norm(rbI));
rcI = rI + transpose(RBI) * rcB;
rpItilde = rpI + wpIk ;
rbItildeu = rbI_u + wbIk ;
zk = [rpItilde', rbItildeu'];
k = 0;
for i = 1 : Nf
    if mcVeck(i) ~= 0
        XiI = rXIMat(i,:)' ;
        viI = XiI - rcI ;
        viI_u = viI ./ norm(viI) ;
        viC_u =  RCB * RBI * viI_u ;
        for j = 1 : 3
            wk_i(j) = wC_i(j+k);
        end
        k = k + 3;
        v_i_Ctildeu = viC_u  + wk_i' ;
        zk = [zk, v_i_Ctildeu'];
    end
end

zk = transpose(zk);
end
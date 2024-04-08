function [rx] = hdCameraSimulator(rXI,S,P)
% hdCameraSimulator : Simulates feature location measurements from the
%                     quad's high-definition camera. 
%
%
% INPUTS
%
% rXI -------- 3x1 location of a feature point expressed in I in meters.
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
% OUTPUTS
%
% rx --------- 2x1 measured position of the feature point projection on the
%              camera's image plane, in pixels.  If the feature point is
%              not visible to the camera (the ray from the feature to the
%              camera center never intersects the image plane), then rx is
%              an empty matrix.
%
%+------------------------------------------------------------------------------+

% Inputs
rI = S.statek.rI ; % 3x1 position of CM in the I frame, in meters
RBI = S.statek.RBI ; % 3x3 direction cosine matrix of B frame wrt I frame
RCB = P.sensorParams.RCB ;
Rc = P.sensorParams.Rc ;
rocB = P.sensorParams.rocB ;
Pc = P.sensorParams.pixelSize ;
K = P.sensorParams.K ;
imagePlaneSize = P.sensorParams.imagePlaneSize ;

% Initialize
K_mat = [K,[0;0;0]] ;
X_Bold = [rXI ; 1] ;
RCI = RCB * RBI ;
rcI = rI + transpose(RBI) * rocB;
t = -RCI * rcI;
RCI_P = [RCI;[0,0,0]];
t_P = [t;1];
P_mat = [RCI_P, t_P] ;
% Compute
x = K_mat * P_mat * X_Bold ;
xc_x = x(1)/x(3) ;
xc_y = x(2)/x(3) ;
xc = [xc_x ; xc_y];
wc = mvnrnd(zeros(2,1), Rc)' ;

% Output
rx = 1/Pc * xc + wc ; % x_tilde

if (abs(rx(1))>(imagePlaneSize(1)/2)) && (abs(rx(2))>(imagePlaneSize(2)/2))
    rx = rx;
else
    rx = [];
end

end
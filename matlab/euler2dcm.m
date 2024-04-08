function [R_BW] = euler2dcm(e)
% euler2dcm : Converts Euler angles phi = e(1), theta = e(2), and psi = e(3)
%             (in radians) into a direction cosine matrix for a 3-1-2 rotation.
%
% Let the world (W) and body (B) reference frames be initially aligned.  In a
% 3-1-2 order, rotate B away from W by angles psi (yaw, about the body Z
% axis), phi (roll, about the body X axis), and theta (pitch, about the body Y
% axis).  R_BW can then be used to cast a vector expressed in W coordinates as
% a vector in B coordinates: vB = R_BW * vW
%
% INPUTS
%
% e ---------- 3-by-1 vector containing the Euler angles in radians: phi =
%              e(1), theta = e(2), and psi = e(3)
%
%
% OUTPUTS
%
% R_BW ------- 3-by-3 direction cosine matrix 
% 
%+------------------------------------------------------------------------------+
  
cPhi = cos(e(1)); sPhi = sin(e(1));
cThe = cos(e(2)); sThe = sin(e(2));
cPsi = cos(e(3)); sPsi = sin(e(3));

R_BW = [cPsi*cThe - sPhi*sPsi*sThe, cThe*sPsi + cPsi*sPhi*sThe, -cPhi*sThe;
        -cPhi*sPsi,                                  cPhi*cPsi,       sPhi;
        cPsi*sThe + cThe*sPhi*sPsi, sPsi*sThe - cPsi*cThe*sPhi,  cPhi*cThe];









function [e] = dcm2euler(R_BW)
% dcm2euler : Converts a direction cosine matrix R_BW to Euler angles phi =
%             e(1), theta = e(2), and psi = e(3) (in radians) for a 3-1-2
%             rotation. If the conversion to Euler angles is singular (not
%             unique), then this function issues an error instead of returning
%             e.
%
% Let the world (W) and body (B) reference frames be initially aligned.  In a
% 3-1-2 order, rotate B away from W by angles psi (yaw, about the body Z
% axis), phi (roll, about the body X axis), and theta (pitch, about the body Y
% axis).  R_BW can then be used to cast a vector expressed in W coordinates as
% a vector in B coordinates: vB = R_BW * vW
%
% INPUTS
%
% R_BW ------- 3-by-3 direction cosine matrix 
%
%
% OUTPUTS
%
% e ---------- 3-by-1 vector containing the Euler angles in radians: phi =
%              e(1), theta = e(2), and psi = e(3).  By convention, these
%              should be constrained to the following ranges: -pi/2 <= phi <=
%              pi/2, -pi <= theta < pi, -pi <= psi < pi.  
% 
%+------------------------------------------------------------------------------+
  
epsilon = 1e-15;
phi = asin(R_BW(2,3));
% Note that if both arguments of the atan2 below are zero, as occurs when
% cos(phi) = 0, then the result is undefined. Thus, for phi = pi/2 + n*pi, for
% n an integer, the 3-1-2 Euler representation is singular.  The intuition
% here is that when the roll angle is pi/2, the first and third rotations
% (about the z and y axes, respectively), have exactly the same effect, and so
% can't be distinguished, leading to a non-unique representation for psi and
% theta; only psi + theta is constrained, but not psi and theta individually.
% This function detects this singularity and issues an error.
if(abs(cos(phi)) < epsilon)
  error('The 312 attitude representation is singular: cos(phi) is near 0');
end
theta = atan2(-R_BW(1,3),R_BW(3,3));
if(theta == pi)
  theta = -pi;
end
psi = atan2(-R_BW(2,1),R_BW(2,2));
if(psi == pi)
  psi = -pi;
end
e = [phi; theta; psi];








function [lat,lon,alt] = ecef2lla(pVec)
% ecef2lla : Convert from a position vector in the Earth-centered, Earth-fixed
%            (ECEF) reference frame to latitude, longitude, and altitude
%            (geodetic with respect to the WGS-84 ellipsoid).
%
%
% INPUTS
%
% pVec ---- 3-by-1 position coordinate vector in the ECEF reference frame,
%           in meters.
%
% OUTPUTS
%
% lat ----- latitude in radians
%
% lon ----- longitude in radians
%
% alt ----- altitude (height) in meters above the ellipsoid
% 
%+------------------------------------------------------------------------------+

navConstants;
e = sqrt((AA^2 - BB^2)/AA^2);
ep = sqrt((AA^2 - BB^2)/BB^2);
x = pVec(1);
y = pVec(2);
z = pVec(3);
lon = atan2(y,x);
p = sqrt(x^2 + y^2);
theta = atan2(z*AA, p*BB);
lat = atan2(z + ep^2*BB*sin(theta)^3, p - e^2*AA*cos(theta)^3);
N = AA / sqrt(1 - e^2*sin(lat)^2);
alt = p/cos(lat) - N;

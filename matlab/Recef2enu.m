function [R] = Recef2enu(r)
% ecef2enu : Generates the rotation matrix used to express a position vector
%            written in ECEF coordinates relative to the ECEF vector r as a
%            vector written in the local east, north, up (ENU) coordinate
%            frame with origin at r. The transformation is based on the WGS-84
%            ellipsoid.
%
%
% INPUTS
%
% r ---------- 3x1 ECEF position marking the origin of the ENU frame, in
%              meters.
%
%
% OUTPUTS
%
% R ---------- 3x3 rotation matrix that maps a relative vector vEcef expressed
%              in ECEF coordinates relative to the vector r to a vector vEnu
%              expressed in the local east, north, up (vertical) coordinate
%              frame with origin at r: vEnu = R*vEcef.  
%
%+------------------------------------------------------------------------------+

[lat,lon,alt] = ecef2lla(r);
Robs = zeros(3,1);
[Robs(1), Robs(2), Robs(3)] = sph2cart(lon, lat, 1);

v_vert = Robs;
v_east = cross([0;0;1],Robs);
v_east = v_east/norm(v_east);
v_north = cross(v_vert, v_east);
v_north = v_north/norm(v_north);

vx = v_vert;
vy = v_east;
vz = v_north;

R = [vy';vz';vx'];

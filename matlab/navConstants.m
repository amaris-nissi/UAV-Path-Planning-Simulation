% Physical constants used for navigation

% Gravitational constant * mass of Earth, meters^3/second^2
GM = 3.986005e14;
% WGS-84 ellipsoid semi-major and semi-minor axes, meters.
AA = 6378137.00000;				
BB = 6356752.3142518;
% WGS-84 ellipsoid eccentricity squared
esquare = (AA^2 - BB^2) / AA^2;	
% Mean Earth rotation rate, rad/sec.
OmegaE = 7.2921151467e-5;
% Speed of light, m/s
cLight = 299792458;
% Scaling factor for conversion from degrees to radians
degrad = pi/180.0;
% Fundamental GPS frequency, Hz
f0_fundamentalGPS = 10.23e6;
% GPS L1 frequency, Hz
fL1GPS = 154 * f0_fundamentalGPS;	
% GPS L1 frequency, Hz
fL2GPS = 120 * f0_fundamentalGPS;	
% GPS L1 wavelength, meters
lambdaGPSL1 = cLight / fL1GPS;	
% GPS L2 wavelength, meters
lambdaGPSL2 = cLight / fL2GPS;	
% dry air specific gas constant (J/kg/K)
scgRd = 287.058;
% water vapor specific gas constant (J/kg/K)
scgRw = 461.402;
% Saastamoinen's two constants
saast_cw = 12.92e-8;
saast_cwp = 371900e-8;
% surface temperature lapse rate (change of temp with altitude, deg. K/m)
stBeta = -0.0065;
% seconds in week
sec_in_week = 604800;
% GPS SV List
gpsSVList = [1:32]';
% SBAS SV List
sbasSVList = [120,124,125,126,127,128,129,133,135,136,137,138,140]';



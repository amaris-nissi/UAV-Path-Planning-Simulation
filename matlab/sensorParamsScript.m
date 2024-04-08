% Loads quadrotor sensor parameters into the structure sensorParams

%-------------- GNSS ---------------
% raB(:,1) holds the 3x1 vector pointing from the body (B) frame origin to the
% the primary GNSS antenna, expressed in meters in the B frame.  raB(:,2) is
% identical but for the secondary antenna.
sensorParams.raB = [0.1013 -0.0887; -0.0004 -0.0004; 0.0472 0.0472];
% 3x1 position of reference antenna expressed in the global (G) frame, in
% meters.  The G frame is identical to the Earth-Centered-Earth-Fixed (ECEF)
% frame.  r0G is also taken to be the origin of the local East-North-Up
% (ENU) frame, denoted L.  
sensorParams.r0G = [-742080.456; -5462030.875; 3198338.956];
% The 3x3 error covariance matrix for the unconstrained precise GNSS (RTK)
% solution for either the primary or secondary antenna, expressed in meters
% in the L frame.
sensorParams.RpL = diag([0.006^2, 0.006^2, 0.012^2]);
% Standard deviation of the angular error in the measured baseline-constrained
% vector from the primary to the secondary antenna, in radians.
sensorParams.sigmab = 0.05; 

%----------- HD Camera -------------
% 3x1 vector pointing from the B frame origin to the camera (C) frame origin
% (the camera center), expressed in meters in the B frame.
sensorParams.rocB = [0.1159; -0.0004; -0.0435];
% 3x3 direction cosine matrix relating the C and B reference frames.  For some
% vector v, vC = RCB*vB.
sensorParams.RCB = euler2dcm([120*pi/180, 0, pi/2]);
% 2x2 error covariance matrix for the Gaussian image noise, in pixels^2
sensorParams.Rc = diag([20^2,20^2]);
% Pixel size, in meters
sensorParams.pixelSize = 2e-6;
% Distance of the image plane along camera z axis, in meters
sensorParams.f = sensorParams.pixelSize*1657.72;
% Camera intrinsic matrix
sensorParams.K = diag([sensorParams.f,sensorParams.f,1]);
% Image plane size (i.e., width and height), in meters, along the camera x and
% y dimensions
sensorParams.imagePlaneSize = sensorParams.pixelSize*[3840,2160];

%------------- IMU -----------------
% IMU sampling interval, in seconds
sensorParams.IMUdelt = 0.005;
% 3x1 position of IMU accelerometer proof mass in the B frame, in meters.
sensorParams.lB = zeros(3,1);
% Accelerometer white noise from accel. spec. sheet, in (milli-g)^2/Hz
sensorParams.Sa = (0.1)^2; 
% Error covariance matrix for accelerometer noise, in (m/s^2)^2
sensorParams.Qa = (9.8/1000)^2*(sensorParams.Sa/sensorParams.IMUdelt)*eye(3);
% Accelerometer bias time constant, in seconds
sensorParams.taua = 100;  
% Accelerometer bias from accel. spec. sheet, in milli-g
sensorParams.sigmaa = 100; 
% Error covariance matrix for accelerometer bias noise, in (m/s^2)^2
sensorParams.alphaa = exp(-sensorParams.IMUdelt/sensorParams.taua); 
sensorParams.Qa2 = (9.8/1000)^2*(sensorParams.sigmaa)^2*(1 - sensorParams.alphaa^2)*eye(3);
% Gyro white noise from spec. sheet, in (deg/s)^2/Hz
sensorParams.Sg = (5e-2)^2; 
% Error covariance matrix for gyro noise, in (rad/s)^2
sensorParams.Qg = (pi/180)^2 * (sensorParams.Sg/sensorParams.IMUdelt)*eye(3);
% Gyro bias time constant, in seconds
sensorParams.taug = 100; 
% Gyro bias from spec. sheet, in (deg/h)
sensorParams.sigmag = 100; 
% Error covariance matrix for gyro bias noise, in (rad/s)^2
sensorParams.alphag = exp(-sensorParams.IMUdelt/sensorParams.taug);
sensorParams.Qg2 = (pi/(3600*180))^2*(sensorParams.sigmag)^2*(1-sensorParams.alphag^2)*eye(3);

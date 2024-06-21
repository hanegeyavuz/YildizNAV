% yildiz_nav_real_time_mpu6000: post-processing integration of MPU-6000 
% IMU and Ekinox GNSS data.
%
% The main goal is to integrate MPU-6000 IMU and Ekinox-D GNSS measurements.
%
% Sensors dataset was generated driving a car through the streets of 
% Turin city (Italy).

% NOTE: System assumes that IMU is aligned with respect to body-frame as
% X-forward, Y-right and Z-down.
%
% NOTE: System assumes that yaw angle (heading) is positive clockwise.


clc
close all
clear
matlabrc


addpath C:\Users\yhane\Desktop\Yildiz_Localization_INS_GNSS\ins
addpath C:\Users\yhane\Desktop\Yildiz_Localization_INS_GNSS\ins-gnss
addpath C:\Users\yhane\Desktop\Yildiz_Localization_INS_GNSS\conversions
addpath C:\Users\yhane\Desktop\Yildiz_Localization_INS_GNSS\performance-analysis
addpath C:\Users\yhane\Desktop\Yildiz_Localization_INS_GNSS\plot


%% CONSTANTS

G =  9.80665;       % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h

%% LOADING DATAS

% rtk

fprintf('Yildiz_NAV: loading reference data... \n')

load ref

%imu

fprintf('Yildiz_NAV: loading MPU-6000 IMU data... \n')

load mpu6000_imu

%gnss

fprintf('Yildiz_NAV: loading Ekinox GNSS data... \n')

load ekinox_gnss

%% SENSOR EXTERNAL PARAMETERS FOR EACH SYSTEM

ekinox_gnss.larm = [-0.369, 0.0, -0.219]'; 

ekinox_gnss.eps = mean(diff(mpu6000_imu.t)) / 2; %  A rule of thumb for choosing eps.

%% NAVIGATION TIME

to = (ref.t(end) - ref.t(1));

fprintf('Yildiz_NAV: navigation time is %.2f minutes or %.2f seconds. \n', (to/60), to)

%% INS/GNSS INTEGRATION

    fprintf('Yildiz_NAV: processing INS/GNSS integration... \n')
    
    % Execute INS/GNSS integration
    % ---------------------------------------------------------------------
    nav_mpu6000 = ins_gnss(mpu6000_imu, ekinox_gnss, 'dcm');
    % ---------------------------------------------------------------------
    
    save nav_mpu6000.mat nav_mpu6000    

    
%% TRAVELED DISTANCE

distance = gnss_distance (nav_mpu6000.lat, nav_mpu6000.lon);

fprintf('Yildiz_NAV: distance traveled by the vehicle is %.2f meters or %.2f km. \n', distance, distance/1000)

%% ANALYSIS OF PERFORMANCE FOR A CERTAIN PART OF THE INS/GNSS DATASET

tmin_rmse = ref.t(1); 
tmax_rmse = ref.t(end); 

% Sincronize REF data to tmin and tmax
idx  = find(ref.t > tmin_rmse, 1, 'first' );
fdx  = find(ref.t < tmax_rmse, 1, 'last' );
if(isempty(idx) || isempty(fdx))
    error('ref: empty index')
end

ref.t       = ref.t    (idx:fdx);
ref.roll    = ref.roll (idx:fdx);
ref.pitch   = ref.pitch(idx:fdx);
ref.yaw     = ref.yaw  (idx:fdx);
ref.lat     = ref.lat  (idx:fdx);
ref.lon     = ref.lon  (idx:fdx);
ref.h       = ref.h    (idx:fdx);
ref.vel     = ref.vel  (idx:fdx, :);

%% INTERPOLATION OF INS/GNSS DATASET

% INS/GNSS estimates and GNSS data are interpolated according to the
% reference dataset.

[nav_i,  ref_n] = navego_interpolation (nav_mpu6000, ref);
[gnss_i, ref_g] = navego_interpolation (ekinox_gnss, ref);

%% NAVIGATION RMSE 

rmse_v = print_rmse (nav_i, gnss_i, ref_n, ref_g, 'MPU-6000 INS/GNSS');

%% RMSE TO CVS FILE

csvwrite('mpu6000.csv', rmse_v);

%% PLOTS    
plot_main(ref, ekinox_gnss, nav_mpu6000, gnss_i, nav_i, ref_g, ref_n);

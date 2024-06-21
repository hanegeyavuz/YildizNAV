%   This MATLAB code simulates the YildizNav system, which integrates an
%   inertial navigation system (INS) with a VOR/DME system.
%   INS data is collected from inertial measurement unit (IMU)
%   sensors, Very high frequency omni-directional range / distance measuring
%   equipment (VOR/DME) data is simulated based on GNSS measurements.

%   Authors: MUHAMMED YAVUZ HANEGE, MEHMET EMRE EYVAZ, KEREM VATANSEVER
%
%   This project is inspired by the "NaveGo" INS/GNSS project.
%
%   Specially thanks to our academic advisor, Dr. "BAHADIR ÇATALBAŞ".
%
%
%
%   Copyright (C) 2014, Rodrigo Gonzalez, all rights reserved.
%
%   This file is part of YildizNav, an open-source MATLAB toolbox for
%   simulation of integrated navigation systems.
%
%   YildizNav is free software: you can redistribute it and/or modify
%   it under the terms of the GNU Lesser General Public License (LGPL)
%   version 3 as published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU Lesser General Public License for more details.
%
%   You should have received a copy of the GNU Lesser General Public
%   License along with this program. If not, see
%   <http://www.gnu.org/licenses/>.
%
%
%   Modifications by Yavuz Hanege <hanegeyavuz@gmail.com>
%   Modifications by Mehmet Emre Eyvaz <mehmetemre.eyvaz@gmail.com>
%   Modifications by Kerem Vatansever <keremvatansver@gmail.com>
%   Date: 05.2024


clc; % Clear command window
clear; % Clear workspace
close all; % Close all figure windows
matlabrc; % Reload MATLAB startup options
format long; % Set double-precision format

% Paths to YildizNav functions
addpath C:\Users\yhane\Desktop\Yildiz_Localization_INS_GNSS
addpath C:\Users\yhane\Desktop\Yildiz_Localization_INS_GNSS\ins
addpath C:\Users\yhane\Desktop\Yildiz_Localization_INS_GNSS\ins-gnss
addpath C:\Users\yhane\Desktop\Yildiz_Localization_INS_GNSS\conversions
addpath C:\Users\yhane\Desktop\Yildiz_Localization_INS_GNSS\performance-analysis
addpath C:\Users\yhane\Desktop\Yildiz_Localization_INS_GNSS\plot
addpath C:\Users\yhane\Desktop\Yildiz_Localization_INS_GNSS\allan-variance
addpath C:\Users\yhane\Desktop\Yildiz_Localization_INS_GNSS\Radio-Navigation
addpath C:\Users\yhane\Desktop\Yildiz_Localization_INS_GNSS\VOR_DME_INS


%% CODE EXECUTION PARAMETERS
%%

INS_VOR_DME    = 'ON';
PLOT        = 'ON';

% Check if variables exist, otherwise set default values
if (~exist('INS_VOR_DME', 'var'))
    INS_VOR_DME = 'OFF';
end
if (~exist('PLOT', 'var'))
    PLOT = 'OFF';
end


%% CONSTANTS
%%
G =  9.80665;       % Gravity constant, m/s^2
D2R = (pi/180);     % Degrees to radians
R2D = (180/pi);     % Radians to degrees
G2T = 1E-4;         % Gauss to Tesla
normalize_angle = @(angle) mod(angle + 360, 720) - 360; % Normalize Function

%% TIME INTERVAL
%%
tmin = 0;     % VOR/DME first time
tmax = 400 ;   % VOR/DME last time

%% REFERENCE
%%
load ref_zedf9p.mat
ref_data = struct2table(ref_data);

% ref data structure
ref.lat = ref_data.Latitude;
ref.lon = ref_data.Longitude;
ref.t = double(ref_data.Time);
ref.h = ref_data.Altitude;
% Serveral fields are missing: roll, pitch, yaw,vel etc.

%% IMU DATA
%%

load imu_xsense_delivers.mat
imu_data = struct2table(imu_data);


% IMU data structure
imu.t = double(imu_data.Time);
imu.fb = [imu_data.Ax imu_data.Ay imu_data.Az ] ;
imu.wb = [imu_data.Gx imu_data.Gy imu_data.Gz ] ;


% IMU frequency
imu.freq = get_freq(imu.t);


%% IMU RAW ORIENTATION
%%

fs = 100;
decim = 1;
% IMU frequency
% imu.freq = get_freq(imu.t);

fuse = imufilter('SampleRate',fs,'DecimationFactor',decim);
q = fuse(imu.fb,imu.wb);
[yaw_gyro, pitch_gyro, roll_gyro] = quat2angle(q);

time = (0:decim:size(imu.fb,1)-1)/fs;

%% GNSS DATA
%%

load zedf9p_gnss_delivers.mat
load gnss_vel_data_delivers.mat

gnss_data = struct2table(gnss_data);
gnss_vel_data = struct2table(gnss_vel_data);


% GNSS data structure
gnss.t = double(gnss_data.Time);
gnss.lat = gnss_data.Latitude;
gnss.lon = gnss_data.Longitude;
gnss.h = double(gnss_data.Altitude);
gnss.vel = [ gnss_vel_data.Velx gnss_vel_data.Vely gnss_vel_data.Velz] ;

% GNSS frequency
gnss.freq = get_freq(gnss.t);

%% VOR-DME INIT
%%

stat.lat = 41.126954;
stat.lon = 29.142890;
vor_dme.h = double(gnss_data.Altitude);
% VOR/DME Velocity Calculation is not effective when using simulated data
vor_dme.vel = [ gnss_vel_data.Velx gnss_vel_data.Vely gnss_vel_data.Velz] ;
vor_dme.t = gnss.t;


%% DISTANCE CALCULATION(m)
%%
% Distance data from DME station
vor_dme.distance = dme_dist(gnss,stat);

%% BEARING CALCULATION(Deg)
%%
% Bearing data from VOR Station
vor_dme.bearing = vor_bearing(gnss,stat);

%% VEHICLE POSITION CALCULATE
%%
% Combining VOR and DME systems
[vor_dme.lat, vor_dme.lon] = radio_navigation_position(vor_dme,stat);


%% DEG TO RAD CONVERSIONS
%%

ref.lat = ref.lat * D2R;
ref.lon = ref.lon * D2R;

gnss.lat = gnss.lat * D2R;
gnss.lon = gnss.lon * D2R;

vor_dme.lat = vor_dme.lat * D2R;
vor_dme.lon = vor_dme.lon * D2R;


%% IMU ERROR PROFILE
%%
% Error profile from Allan variance

% Apply to system just one time for taking error profile

% imu_allan = allan_imu(imu);
%


imu.ini_align = [0 0 0];%[-9.9864e-05 0.0103 0.2256];
imu.ini_align_err = [0.5 0.5 1.5] * D2R;
imu.vrw = [0.006466174857229,0.006746788307829,0.152643335870150];
imu.arw = [0.364802707714128,0.274629627774925,0.072794820486738];
%imu.vrrw =[0.406802865259831,0.625634626023917,0.314003046265368];
%imu.arrw = [4.740911168128429,0.015049942553810,0.001369993101230];
imu.ab_dyn = [3.92266e-04 3.92266e-04 3.92266e-04];%[0.102228362094025,0.178117706218888,0.081545508748972];
imu.gb_dyn = [2.9088821e-05 2.9088821e-05 2.9088821e-05];%[0.169951188758752,0.204961016488104,0.017703685490591];
imu.ab_corr = [0.149955337204109,0.139958314723835,0.119964269763287];
imu.gb_corr = [12.996129224356110,0.989705225547119,170.8191231204407];
imu.ab_psd = [0 0 0];%[6.864655e-04 6.864655e-04 6.864655e-04];%[0.039586979522962,0.066635620459271,0.028243987065788];%[1.4930e-03  1.5890e-03 1.7930e-03];%[0.039586979522962,0.066635620459271,0.028243987065788];
imu.gb_psd = [0 0 0];%[5.236e-05 5.236e-05 5.236e-05];%[0.612676492204525,0.203903273420657,0.231383256855385];%[1.4364e-03 1.3617e-03 1.5617e-03];%[0.612676492204525,0.203903273420657,0.231383256855385];
imu.ab_sta = [4e-06 4e-06 4e-06];
imu.gb_sta = [2.90888e-06 2.90888e-06 2.90888e-06];



%% VOR/DME ERROR PROFILE
%%
vor_dme.stdm = [0.01 0.001 0.01]; % Standart Deviation Position Error
vor_dme.stdv = ones(1,3) .* 0.1 ; % Standart Deviation Velocity Error
vor_dme.zupt_th  = 0.1;       % ZUPT threshold (m/s).
vor_dme.zupt_win = 2;         % ZUPT time window (seconds)
vor_dme.eps = mean(diff(imu.t)) / 2;  % Time Interval
vor_dme = gnss_m2r(vor_dme.lat(1), vor_dme.h(1), vor_dme);
vor_dme.larm = [0 0.03974 0.00851]'; % Same Larm with GNSS

%% NAVIGATION TIME
%%
to = (gnss.t(end) - gnss.t(1));

fprintf('YildizNav: navigation time is %.2f minutes or %.2f seconds. \n', (to/60), to)

%% TRAVELED DISTANCE
%%
distance = gnss_distance (gnss.lat, gnss.lon);

fprintf('YildizNav: distance traveled by the vehicle is %.2f meters or %.2f km. \n', distance, distance/1000)

%% INS/VOR/DME INTEGRATION
%%

if strcmp(INS_VOR_DME, 'ON')

    % Execute INS/VOR/DME integration
    %     ---------------------------------------------------------------------
    fprintf('YildizNav: INS/VOR/DME integration... \n')
    yildiz_nav = ins_vor_dme(imu, vor_dme, 'dcm');
    % ---------------------------------------------------------------------

    save yildiz_nav yildiz_nav
else

    load yildiz_nav
end


% Referance Orientation is not provided by RTK. Equations for init
% orientation of ref structure.
ref.yaw = yildiz_nav.yaw;
ref.roll = yildiz_nav.roll;
ref.pitch = yildiz_nav.pitch;

%% INTERPOLATION OF INS/VOR/DME DATASET
%%

% INS/VOR/DME estimates and GNSS data are interpolated according to the
% reference dataset.

[nav_i,  ref_n] = interpolation (yildiz_nav, ref);
[gnss_i, ref_g] = interpolation (gnss, ref);

%% NAVIGATION RMSE
%%
% RMS Calculation

rmse_v = print_rmse (nav_i, gnss_i, ref_n, ref_g, 'Inertial Sense INS/VOR/DME');

%% PLOTS
%%

if (strcmp(PLOT,'ON'))

    % Main Plot Sequence
    plot_main (ref, gnss, yildiz_nav, gnss_i, nav_i, ref_g, ref_n);

    % Plot for compare gyro and INS/VOR/DME Outputs
    %roll_nav = normalize_angle(R2D .* (yildiz_nav.roll));
    roll_nav = normalize_angle(R2D .* (yildiz_nav.roll));
    pitch_nav = normalize_angle(R2D .*(yildiz_nav.pitch));
    yaw_nav = normalize_angle(R2D .*(yildiz_nav.yaw));

    roll_gyro = normalize_angle(R2D .*(roll_gyro));
    pitch_gyro = normalize_angle(R2D .*(pitch_gyro));
    yaw_gyro = normalize_angle(R2D .*(yaw_gyro));


    % Raw imu sensor orientation
    figure;
    plot(time,[yaw_gyro,pitch_gyro,roll_gyro])
    title('Raw IMU Orientation Estimate')
    legend('Z-axis', 'Y-axis', 'X-axis')
    xlabel('Time (s)')
    ylabel('Rotation (Deg)')


    % Comparison Sequence
    figure;
    subplot(3,1,1);
    plot(time, roll_nav, 'b', time, roll_gyro, 'r');
    xlabel('Time (s)');
    ylabel('Roll (Deg)');
    legend('INS/VOR/DME', 'Gyro');
    title('Roll');

    subplot(3,1,2);
    plot(time, pitch_nav, 'b', time, pitch_gyro, 'r');
    xlabel('Time (s)');
    ylabel('Pitch (Rad)');
    legend('INS/VOR/DME', 'Gyro');
    title('Pitch');

    subplot(3,1,3);
    plot(time, yaw_nav, 'b', time, yaw_gyro, 'r');
    xlabel('Time (s)');
    ylabel('Yaw (Deg)');
    legend('INS/VOR/DME', 'Gyro');
    title('Yaw');
end





clc
clear
close all
matlabrc
format long;
%%

G =  9.80665;       % Gravity constant, m/s^2
D2R = (pi/180);     % Degrees to radians
R2D = (180/pi);     % Radians to degrees
G2T = 1E-4;         % Gauss to Tesla

%% Load GNSS Data for Vehicle Position
load zedf9p_gnss_delivers.mat
load('imu_xsense_delivers.mat')
load('ekinox_gnss.mat')

%% Init VOR Station Position

stat.lat = 44.925400;
stat.lon = 7.861660;

%% GNSS structure

% num_samples = length(gnss_data);
% 
% for i = 1:num_samples
%     gnss.lat(i) = (gnss_data(i).Latitude)';
%     gnss.lon(i) = (gnss_data(i).Longitude)';
%     gnss.t(i) = (gnss_data(i).Time)';
%     gnss.h(i) = (gnss_data(i).Altitude)';
% 
% end

gnss.lat = ekinox_gnss.lat * R2D;
gnss.lon = ekinox_gnss.lon * R2D;
vor_dme.t = double((ekinox_gnss.t));
vor_dme.h = ekinox_gnss.h;
gnss.h = ekinox_gnss.h;


%% DISTANCE Calculation(m)

vor_dme.distance = dme_dist(gnss,stat);

%% BEARING Calculation(Deg)

vor_dme.bearing = vor_bearing(gnss,stat);


%% Vehicle Position Calculate

[vor_dme.lat, vor_dme.lon] = radio_navigation_position(vor_dme,stat);

 gnss.lon = gnss.lon * D2R;
 gnss.lat = gnss.lat * D2R;
 
 vor_dme.lat = (vor_dme.lat) * D2R;
 vor_dme.lon = (vor_dme.lon) * D2R;


%% PLOT

% GNSS verilerini görselleştirme
figure;
plot(gnss.lon, gnss.lat ,'b.', 'MarkerSize', 5);
hold on;
plot(vor_dme.lon, vor_dme.lat, 'r-', 'MarkerSize', 1);
legend('GNSS Data', 'VOR/DME Station');
xlabel('Longitude');
ylabel('Latitude');
title('GNSS Data and VOR/DME Station');
grid on;
axis equal; % x ve y eksenlerinin ölçeklerini eşit hale getirir
hold off;

[RM,RN] = radius(gnss.lat);
LAT2M = (RM + gnss.h);                     % Coefficient for lat, radians to meters
LON2M = (RN + gnss.h) .* cos(gnss.lat);   % Coefficient for lon, radians to meters

RMSE_lat = rmse (vor_dme.lat.* LAT2M, gnss.lat.* LAT2M) ;
RMSE_lon = rmse (vor_dme.lon.* LON2M, gnss.lon.* LON2M) ;
RMSE_h   = rmse (vor_dme.h, gnss.h);












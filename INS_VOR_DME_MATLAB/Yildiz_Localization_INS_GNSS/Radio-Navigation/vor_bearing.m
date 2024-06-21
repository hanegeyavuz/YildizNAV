function [bearing] = vor_bearing(gnss,vor)

% DESCRIPTION:
% This function is simulate VOR Receiver via using GNSS measurements.


% INPUT

% gnss, GNSS sensor structure
% gnss.lat, Vehicle latitude
% gnss.lon, Vehicle Longitude
% gnss.slat, Station Latitude
% gnss.slon, Station Longitude

D2R = (pi/180);     % Degrees to radians
R2D = (180/pi);     % Radians to degrees



% GNSS ve VOR konumlarını radyan cinsine çevir
gnss.lat = (gnss.lat)* D2R;
gnss.lon = (gnss.lon)* D2R;
vor.lat = (vor.lat)* D2R;
vor.lon = (vor.lon)* D2R;

% Boylam farkını hesapla
dlon = gnss.lon - vor.lon;

% x ve y bileşenlerini hesapla
x = sin(dlon) .* cos(gnss.lat);
y = cos(vor.lat) .* sin(gnss.lat) - sin(vor.lat) .* cos(gnss.lat) .* cos(dlon);

% Bearing (azimut) hesapla
bearing = atan2(x, y);

% Bearing'i derece cinsine çevir ve 0-360 aralığına getir
bearing = (bearing)* R2D;
bearing = mod(bearing + 360, 360); % 0-360 aralığına getirilmesi
end


function [distance] = dme_dist(gnss,dme)

D2R = (pi/180);     % Degrees to radians
R2D = (180/pi);     % Radians to degrees


% INPUT
% dme, DME station coordinates structure.
% dme.lat,  DME station latitude,
% dme.lon, DME station longitude

% OUTPUT
% distance, Distance between DME Station and Vehicle


gnss.lat = (gnss.lat)* D2R;
gnss.lon = (gnss.lon)* D2R;
dme.lat = (dme.lat)* D2R;
dme.lon = (dme.lon) * D2R;

delta_lat = (gnss.lat-(dme.lat));
delta_lon = (gnss.lon-(dme.lon));

R = sqrt(6378137 * 6356752.31424518); 

a = sin( delta_lat ./ 2 ).^2 + cos( (dme.lat) ).* cos( gnss.lat ) .* ...
        sin( delta_lon ./ 2 ).^2;

% Ensure that a falls in the closed interval [0 1].
a(a < 0) = 0;
a(a > 1) = 1;

c = 2 .* atan2 ( real(sqrt(a)), real(sqrt(1-a)) );

delta_pos = R .* c;

distance = (delta_pos);

end
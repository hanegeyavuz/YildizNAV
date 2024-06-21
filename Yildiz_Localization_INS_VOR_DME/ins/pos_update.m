function pos = pos_update(pos, vel, dt)
% pos_update: updates position in the navigation frame (lat, lon, h).
%
% INPUT
%   pos,  3x1 position vector [lat lon h] (rad, rad, m).
%   vel,  3x1 NED velocities [n e d] (m/s).
%   dt,   sampling interval (s).
%
% OUTPUT
%   pos,    3x1 updated position vector [lat lon h] (rad, rad, m).


pos = double(pos);
format long;

lat = pos(1); 
lon = pos(2); 
h   = pos(3);
vn  = vel(1);
ve  = vel(2);
vd  = vel(3);

%% Altitude

h  = h - vd * dt;

if h < 0.0
    h = abs(h);
    %warning('pos_update: altitude is negative.')
end

%% Latitude

[RM, ~] = radius(lat);

vn = vn / (RM + h);

dlat = vn * dt;

% Ensure lat is of type single or double before passing to deg2rad
if ~isa(lat, 'single') && ~isa(lat, 'double')
    lat = double(lat);
end

lat = lat + vn * dlat;

%% Longitude

[~, RN] = radius(lat);

ve  = ve / ((RN + h) * cos(double(lat)));

dlon = ve * dt;

% Ensure lon is of type single or double before passing to deg2rad
if ~isa(lon, 'single') && ~isa(lon, 'double')
    lon = double(lon);
end

lon = lon + ve * dlon;

%% Position update

pos = [lat lon h];

end

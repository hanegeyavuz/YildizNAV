function [vel] = vor_dme_velocity(vor_dm)
%VOR_DME_VELOCİTY Calculate velocity in NED coordinate frame via VOR_DME
%Position
%   INPUT
%   vor_dm, VOR/DME Station Structure.
%   vor_dm.lat, Latitude of Vehicle
%   vor_dm.lon, Longitude of Vehicle                                  
%   vor_dm.h = Altitude of Vehicle
%
%   OUTPUT
%   vn, Velocity in North
%   ve, Velocity in East
%   vd, Velocity in Down


% Remove duplicate or non-increasing time entries
[unique_time, unique_idx] = unique(vor_dm.t, 'stable');

% Filter corresponding position values
unique_lat = vor_dm.lat(unique_idx);
unique_lon = vor_dm.lon(unique_idx);
unique_h = vor_dm.h(unique_idx);

% Ensure there are no Inf or NaN values in the filtered inputs
if any(isnan(unique_lat)) || any(isnan(unique_lon)) || any(isnan(unique_h))
    error('Position vectors contain NaN values');
end
if any(isinf(unique_lat)) || any(isinf(unique_lon)) || any(isinf(unique_h))
    error('Position vectors contain Inf values');
end

% Calculate gradients
vel.V_L = gradient(unique_lat, unique_time);  % Lat Vel
vel.V_p = gradient(unique_lon, unique_time);  % Lon Vel
vel.V_h = gradient(unique_h, unique_time);    % H Vel

% Convert latitude to radians for NED conversion
lat_rad = deg2rad(unique_lat);  % Deg to Rad for convert to NED

% Convert to NED velocities
vel.vn = -vel.V_L';                         
vel.ve = (vel.V_p .* cos(lat_rad))';
vel.vd = (-vel.V_h)';

end


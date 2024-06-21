function [RM,RN] = radius(lat)
% radius: calculates meridian and normal radii of curvature.
%
% INPUT
%   lat, Nx1 latitude (rad).
%
% OUTPUT
%   RM, Nx1 meridian radius of curvature (North-South)(m).
%   RN, Nx1 normal radius of curvature (East-West) (m).


a = 6378137.0;                  % WGS84 Equatorial radius in meters
e = 0.0818191908425;            % WGS84 eccentricity

e2 = e^2;
den = 1 - e2 .* (sin(double(lat))).^2;

% Meridian radius of curvature: radius of curvature for North-South motion.
RM = a .* (1-e2) ./ (den).^(3/2);

% Normal radius of curvature: radius of curvature for East-West motion.
% AKA transverse radius.
RN = a ./ sqrt(den);

end

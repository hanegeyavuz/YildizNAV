function gn = gravity(lat, h)
% gravity: calculates gravity vector in the navigation frame.
%
% INPUT
%       lat, Mx1 latitude (radians).
%         h, Mx1 altitude (m).
%
% OUTPUT
%		g_n, Mx3 gravity vector in the nav-frame [m/s^2 m/s^2 m/s^2].

M = max(size(lat));

gn = zeros(M,3);

%   RM, Nx1 meridian radius of curvature (North-South)(m).
%   RN, Nx1 normal radius of curvature (East-West) (m).

% Parameters
RN = 6378137;               % WGS84 Equatorial radius in meters
RM = 6356752.31425;         % WGS84 Polar radius in meters
e = 0.0818191908425;        % WGS84 eccentricity
f = 1 / 298.257223563;      % WGS84 flattening
mu = 3.986004418E14;        % WGS84 Earth gravitational constant (m^3 s^-2)
omega_ie_n = 7.292115E-5;   % Earth rotation rate (rad/s)

% Calculate surface gravity using the Somigliana model, (2.134)
sinl2 = sin(lat).^2;
g_0 = 9.7803253359 * (1 + 0.001931853 .* sinl2) ./ sqrt(1 - e^2 .* sinl2);

% Calculate north gravity using (2.140)
gn(:,1) = -8.08E-9 .* h .* sin(2 .* lat);

% East gravity is zero
gn(:,2) = 0;

% Calculate down gravity using (2.139)
gn(:,3) = g_0 .* (1 - (2 ./ RN) .* (1 + f .* (1 - 2 .* sinl2) +...
    (omega_ie_n^2 .* RN^2 .* RM ./ mu)) .* h + (3 .* h.^2 ./ RN^2));

end


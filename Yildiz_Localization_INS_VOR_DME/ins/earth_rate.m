function omega_ie_n = earth_rate(lat)
% earth_rate: turn rate of the Earth in the navigation frame.
%
% INPUT
%	lat, 1x1 latitude (rad).
%
% OUTPUT
%	omega_ie_n, 3x3 skew-symmetric Earth rate matrix (rad/s).


omega_ie_n = (7.2921155e-5) .* [0 sin(lat) 0; -sin(lat) 0 -cos(lat); 0 cos(lat) 0;]; 

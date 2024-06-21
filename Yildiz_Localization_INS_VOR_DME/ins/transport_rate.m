function omega_en_n = transport_rate(lat, Vn, Ve, h)
% transport_rate: calculates the transport rate in the navigation frame.
%
% INPUT
%	lat, 1x1 latitude (rad).
%	Vn, 1x1 North velocity (m/s).
%   Ve, 1x1 East velocity (m/s).
%   h, altitude (m).
%
% OUTPUT
%	omega_en_n, 3x3 skew-symmetric transport rate matrix (rad/s).

h = abs(h);


[RM,RN] = radius(lat);

om_en_n(1,1) =   Ve / (RN + h);              % North
om_en_n(2,1) = -(Vn / (RM + h));             % East
om_en_n(3,1) = -(Ve * tan(lat) / (RN + h));  % Down

omega_en_n = skewm(om_en_n);

end

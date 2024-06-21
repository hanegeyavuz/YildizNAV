function rmse_v = yildiz_rmse (nav, gnss, ref_n, ref_g)
% yildiz_rmse: calculates the Root Mean Squared Errors (RMSE) between 
% a INS/GNSS system and a reference data structure, and between GNSS-only 
% solution and a reference data structure.
%
% INPUT
%   nav_e, INS/GNSS integration data structure.
%   gnss,  GNSS data structure.
%   ref_n, Reference data structure ajusted for INS/GNSS estimations.
%   ref_g, Reference data structure ajusted for GNSS measurements.
%
% OUTPUT
%   rmse_v, vector with all RMSE.
%       RMSE_roll;  RMSE_pitch; RMSE_yaw; (degrees, degrees, degrees)    
%       RMSE_vn;    RMSE_ve;    RMSE_vd;  (m/s, m/s, m/s) 
%       RMSE_lat;   RMSE_lon;   RMSE_h;   (m, m, m)
%       RMSE_vn_g;  RMSE_ve_g;  RMSE_vd_g;(m/s, m/s, m/s)
%       RMSE_lat_g; RMSE_lon_g; RMSE_h_g; (m, m, m)


D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

%% INS/GNSS ATTITUDE RMSE

RMSE_roll  = rmse (nav.roll , ref_n.roll)  .* R2D;
RMSE_pitch = rmse (nav.pitch, ref_n.pitch) .* R2D;

% Differences greater than 300 deg are avoided when comparing yaw angles.
% The idea is to avoid to compare values of yaw angles when, for example, the 
% reference yaw is near pi and the nav yaw is near -pi. Both yaw angles, pi
% and -pi, represent the same heading angle (moving South).

nav.yaw = correct_yaw(nav.yaw);
ref_n.yaw = correct_yaw(ref_n.yaw);

idx = ( abs(nav.yaw - ref_n.yaw) < (300 * D2R) );
RMSE_yaw = rmse ( nav.yaw(idx), ref_n.yaw(idx) ) .* R2D;

%% INS/GNSS VELOCITY RMSE

if (isfield(nav, 'vel') && isfield(ref_n, 'vel'))
    RMSE_vn = rmse (nav.vel(:,1),  ref_n.vel(:,1));
    RMSE_ve = rmse (nav.vel(:,2),  ref_n.vel(:,2));
    RMSE_vd = rmse (nav.vel(:,3),  ref_n.vel(:,3));
else
    RMSE_vn = NaN;
    RMSE_ve = NaN;
    RMSE_vd = NaN;
    %warning('navego_rmse: no NED velocity field was found in INS/GNSS or Ref data.');
end

%% INS/GNSS POSITION RMSE

[RM,RN] = radius(ref_n.lat);
LAT2M = (RM + ref_n.h);                     % Coefficient for lat, radians to meters
LON2M = (RN + ref_n.h) .* cos(ref_n.lat);   % Coefficient for lon, radians to meters

RMSE_lat = rmse (nav.lat.* LAT2M, ref_n.lat.* LAT2M) ;
RMSE_lon = rmse (nav.lon.* LON2M, ref_n.lon.* LON2M) ;
RMSE_h   = rmse (nav.h, ref_n.h);

%% GNSS VELOCITY RMSE

if (isfield(gnss, 'vel') && isfield( ref_g, 'vel'))
    RMSE_vn_g = rmse (gnss.vel(:,1), ref_g.vel(:,1));
    RMSE_ve_g = rmse (gnss.vel(:,2), ref_g.vel(:,2));
    RMSE_vd_g = rmse (gnss.vel(:,3), ref_g.vel(:,3));
else
    RMSE_vn_g = NaN;
    RMSE_ve_g = NaN;
    RMSE_vd_g = NaN;
    %warning('navego_rmse: no NED velocity field was found in GNSS or REF data.');
end

%% GNSS POSITION RMSE

[RMg,RNg] = radius(ref_g.lat);
LAT2Mg = (RMg + ref_g.h);                   % Coefficient for lat, radians to meters
LON2Mg = (RNg + ref_g.h) .* cos(ref_g.lat); % Coefficient for lon, radians to meters

RMSE_lat_g = rmse (gnss.lat.* LAT2Mg, ref_g.lat.* LAT2Mg) ;
RMSE_lon_g = rmse (gnss.lon.* LON2Mg, ref_g.lon.* LON2Mg) ;
RMSE_h_g   = rmse (gnss.h, ref_g.h);

%%

rmse_v = [  RMSE_roll;  RMSE_pitch; RMSE_yaw;    
            RMSE_vn;    RMSE_ve;    RMSE_vd;
            RMSE_lat;   RMSE_lon;   RMSE_h;
            RMSE_vn_g;  RMSE_ve_g;  RMSE_vd_g;
            RMSE_lat_g; RMSE_lon_g; RMSE_h_g; ];
end
        
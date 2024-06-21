function [nav_e] = ins_vor_dme(imu, vor_dme, att_mode)
% ins_vor_dme: loosely-coupled integrated navigation system.
%
% ins_vor_dme integrates IMU and vor_dme measurements by using an Extended Kalman filter.
%
% INPUT
%   imu, IMU data structure.
%         t: Ix1 time vector (seconds).
%        fb: Ix3 accelerations vector in body frame XYZ (m/s^2).
%        wb: Ix3 turn rates vector in body frame XYZ (radians/s).
%       arw: 1x3 angle random walks (rad/s/root-Hz).
%       vrw: 1x3 velocity random walks (m/s^2/root-Hz).
%     g_std: 1x3 gyros standard deviations (radians/s).
%     a_std: 1x3 accrs standard deviations (m/s^2).
%    gb_sta: 1x3 gyros static biases or turn-on biases (radians/s).
%    ab_sta: 1x3 accrs static biases or turn-on biases (m/s^2).
%    gb_dyn: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%    ab_dyn: 1x3 accrs dynamic biases or bias instabilities (m/s^2).
%   gb_corr: 1x3 gyros correlation times (seconds).
%   ab_corr: 1x3 accrs correlation times (seconds).
%    gb_psd: 1x3 gyros dynamic biases root-PSD (rad/s/root-Hz).
%    ab_psd: 1x3 accrs dynamic biases root-PSD (m/s^2/root-Hz);
%      freq: 1x1 sampling frequency (Hz).
% ini_align: 1x3 initial attitude at t(1).
% ini_align_err: 1x3 initial attitude errors at t(1).
%
%   vor_dme, VOR/DME data structure.
%         t: Gx1 time vector (seconds).
%       lat: Gx1 latitude (radians).
%       lon: Gx1 longitude (radians).
%         h: Gx1 altitude (m).
%       vel: Gx3 NED velocities (m/s).
%       std: 1x3 position standard deviations (rad, rad, m).
%      stdm: 1x3 position standard deviations (m, m, m).
%      stdv: 1x3 velocity standard deviations (m/s).
%      larm: 3x1 lever arm from IMU to vor_dme antenna (x-fwd, y-right, z-down) (m).
%      freq: 1x1 sampling frequency (Hz).
%   zupt_th: 1x1 ZUPT threshold (m/s).
%  zupt_win: 1x1 ZUPT time window (seconds).
%       eps: 1x1 time interval to compare current IMU time to current vor_dme time vector (s).
%
%
%	vor_dme, vor_dme data structure.
%         t: Gx1 time vector (seconds).
%       lat: Gx1 latitude (radians).
%       lon: Gx1 longitude (radians).
%         h: Gx1 altitude (m).
%       vel: Gx3 NED velocities (m/s).
%       std: 1x3 position standard deviations (rad, rad, m).
%      stdm: 1x3 position standard deviations (m, m, m).
%      stdv: 1x3 velocity standard deviations (m/s).
%      larm: 3x1 lever arm from IMU to vor_dme antenna (x-fwd, y-right, z-down) (m).
%      freq: 1x1 sampling frequency (Hz).
%   zupt_th: 1x1 ZUPT threshold (m/s).
%  zupt_win: 1x1 ZUPT time window (seconds).
%       eps: 1x1 time interval to compare current IMU time to current vor_dme time vector (s).
%
%  att_mode: attitude mode string.
%      'quaternion': attitude updated in quaternion format. Default value.
%             'dcm': attitude updated in Direct Cosine Matrix format.
%
% OUTPUT
%   nav_e, INS/vor_dme navigation estimates data structure.
%         t: Ix1 INS time vector (seconds).
%        tg: Gx1 vor_dme time vector, when Kalman filter was executed (seconds).
%      roll: Ix1 roll (radians).
%     pitch: Ix1 pitch (radians).
%       yaw: Ix1 yaw (radians).
%       vel: Ix3 NED velocities (m/s).
%       lat: Ix1 latitude (radians).
%       lon: Ix1 longitude (radians).
%         h: Ix1 altitude (m).
%        xi: Gxn Kalman filter a priori states.
%        xp: Gxn Kalman filter a posteriori states.
%         z: Gxr INS/vor_dme measurements
%         v: Gxr Kalman filter innovations.
%         b: Gxr Kalman filter biases compensations, [gb_dyn ab_dyn].
%         A: Gxn^2 Kalman filter transition-state matrices, one matrix per
%            row ordered by columns.
%        Pp: Gxn^2 Kalman filter a posteriori covariance matrices, one
%         matrix per row ordered by columns.
%        Pi: Gxn^2 Kalman filter a priori covariance matrices, one matrix
%            per row ordered by columns.
%         K: Gx(n*r) Kalman gain matrices
%         S: Gxr^2 Innovation matrices
%        ob: Gx1 Number of observable states after each vor_dme data arriving



if nargin < 3, att_mode  = 'quaternion'; end

%% ZUPT ALGORITHM

zupt_flag = false;

%% PREALLOCATION

% Kalman filter dimensions
n = 15; % number of states
r = 6;  % number of sensors
% q = 12; % number of inputs

% Constant matrices
I = eye(3);
O = zeros(3);

% Length of INS time vector
LI = length(imu.t);

% Length of vor_dme time vector
LG = length(vor_dme.t);

% Preallocation of attitude vectors
roll_e  = zeros (LI, 1);
pitch_e = zeros (LI, 1);
yaw_e   = zeros (LI, 1);

% Preallocation of velocity vector
vel_e   = zeros (LI, 3);

% Preallocation of gravity vector
gn_e   = zeros (LI, 3);

% Preallocation of position vectors
lat_e    = zeros (LI, 1);
lon_e    = zeros (LI, 1);
h_e      = zeros (LI, 1);

% Preallocation of Kalman filter matrices for later performance analysis
xi = zeros(LG, n);      % Evolution of Kalman filter a priori states
xp = zeros(LG, n);      % Evolution of Kalman filter a posteriori states
z = zeros(LG, r);       % INS/vor_dme measurements
v = zeros(LG, r);       % Kalman filter innovations

A  = zeros(LG, n^2);    % Transition-state matrices
Pi = zeros(LG, n^2);    % A priori covariance matrices
Pp = zeros(LG, n^2);    % A posteriori covariance matrices
K  = zeros(LG, n*r);    % Kalman gain matrices
S  = zeros(LG, r^2);    % Innovation matrices
ob = zeros(LG, 1);      % Number of observable states at each vor_dme data arriving

b = zeros(LG, r);       % Biases compensantions after Kalman filter correction

%% INITIAL VALUES AT INS TIME = 1

% Initial attitude
roll_e(1)  = imu.ini_align(1);
pitch_e(1) = imu.ini_align(2);
yaw_e(1)   = imu.ini_align(3);
DCMnb = euler2dcm([roll_e(1); pitch_e(1); yaw_e(1);]);
DCMbn = DCMnb';
qua   = euler2qua([roll_e(1) pitch_e(1) yaw_e(1)]);

% Initial velocity
vel_e(1,:) = vor_dme.vel(1,:);

% Initial position
lat_e(1) = vor_dme.lat(1);
lon_e(1) = vor_dme.lon(1);
h_e(1)   = vor_dme.h(1);

% Initial dynamic biases
gb_dyn = imu.gb_dyn';
ab_dyn = imu.ab_dyn';

% Turn-rates update with both updated velocity and position
omega_ie_n = earth_rate(lat_e(1));
omega_en_n = transport_rate(lat_e(1), vel_e(1,1), vel_e(1,2), h_e(1));

% Gravity update
gn_e(1,:) = gravity(lat_e(1), h_e(1));

%% INITIALIZATION OF KALMAN FILTER MATRICES

% Prior estimates
kf.xi = [ zeros(1,9), imu.gb_dyn, imu.ab_dyn ]';  % Error vector state
kf.Pi = diag([imu.ini_align_err, vor_dme.stdv, vor_dme.std, imu.gb_dyn, imu.ab_dyn].^2);

kf.Q  = diag([imu.arw, imu.vrw, imu.gb_psd, imu.ab_psd].^2);
%kf.Q  = diag([imu.arw, imu.vrw, [0 0 0], [0 0 0]].^2);

fn = DCMbn * (imu.fb(1,:)' - ab_dyn - imu.ab_sta');
wn = DCMbn * (imu.wb(1,:)' - gb_dyn - imu.gb_sta');

% Vector to update matrix F
upd = [vor_dme.vel(1,:) vor_dme.lat(1) vor_dme.h(1) fn' wn'];

% Update matrices F and G
[kf.F, kf.G] = F_update(upd, DCMbn, imu);

[RM,RN] = radius(vor_dme.lat(1));
Tpr = diag([(RM + vor_dme.h(1)), (RN + vor_dme.h(1)) * cos(vor_dme.lat(1)), -1]);  % radians-to-meters

% Update matrix H
kf.H = [ O I O O O ; O O Tpr O O ; ];
kf.R = diag([vor_dme.stdv vor_dme.stdm]).^2;
kf.z = [ vor_dme.stdv, vor_dme.stdm ]';

% Propagate prior estimates to get xp(1) and Pp(1)
kf = kf_update( kf );

% Initial matrices for Kalman filter performance analysis
xi(1,:) = kf.xi';
xp(1,:) = kf.xp';
Pi(1,:) = reshape(kf.Pi, 1, n^2);
Pp(1,:) = reshape(kf.Pp, 1, n^2);
K(1,:)  = reshape(kf.K, 1, n*r);
S(1,:)  = reshape(kf.S, 1, r^2);
v(1,:)  = kf.v';
z(1,:)  = kf.z';
b(1,:) = [gb_dyn', ab_dyn'];

%% INS (IMU) TIME IS THE MASTER CLOCK
for i = 2:LI

    %% INERTIAL NAVIGATION SYSTEM (INS)

    % Print a dot on console every 10,000 INS executions
    if (mod(i,10000) == 0), fprintf('. ');  end
    % Print a return on console every 200,000 INS executions
    if (mod(i,200000) == 0), fprintf('\n'); end

    % IMU sampling interval
    dti = imu.t(i) - imu.t(i-1);

    % Inertial sensors corrected with a posteriori KF biases estimation and
    % deterministic static biases
    wb_corrected = imu.wb(i,:)' - gb_dyn - imu.gb_sta';
    fb_corrected = imu.fb(i,:)' - ab_dyn - imu.ab_sta';
    fn = DCMbn * fb_corrected;
    wn = DCMbn * wb_corrected;


    % Velocity update
    vel = vel_update(fn, vel_e(i-1,:), omega_ie_n, omega_en_n, gn_e(i-1,:)', dti);
    vel_e (i,:) = vel;

    % Position update
    pos = double(pos_update([lat_e(i-1) lon_e(i-1) h_e(i-1)], vel_e(i,:), dti));
    lat_e(i) = double(pos(1));
    lon_e(i) = double(pos(2));
    h_e(i)   = double(pos(3));

    % Turn-rates update with both updated velocity and position
    omega_ie_n = earth_rate(lat_e(i));
    omega_en_n = transport_rate(lat_e(i), vel_e(i,1), vel_e(i,2), h_e(i));

    % Gravity update
    gn_e(i,:) = gravity(lat_e(i), h_e(i));

    % Attitude update
    [qua, DCMbn, euler] = att_update(wb_corrected, DCMbn, qua, ...
        omega_ie_n, omega_en_n, dti, att_mode);
    roll_e(i) = euler(1);
    pitch_e(i)= euler(2);
    yaw_e(i)  = euler(3);

    %% ZUPT DETECTION ALGORITHM
    idz = floor( vor_dme.zupt_win / dti ); % Index to set ZUPT window time

    if ( i > idz )

        % Mean velocity value for the ZUPT window time
        vel_m = mean (vel_e(i-idz:i , :));

        % If mean velocity value is under the ZUPT threshold velocity...
        if (abs(vel_m) < vor_dme.zupt_th)

            % Current attitude is equal to the mean of previous attitudes
            % inside the ZUPT window time
            roll_e(i)  = mean (roll_e(i-idz:i , :));
            pitch_e(i) = mean (pitch_e(i-idz:i , :));
            yaw_e(i)   = mean (yaw_e(i-idz:i , :));

            % Current position is equal to the mean of previous positions
            % inside the ZUPT window time
            lat_e(i) = mean (lat_e(i-idz:i , :));
            lon_e(i) = mean (lon_e(i-idz:i , :));
            h_e(i)   = mean (h_e(i-idz:i , :));

            % Alternative attitude ZUPT correction
            % roll_e(i)  = (roll_e(i-idz , :));
            % pitch_e(i) = (pitch_e(i-idz , :));
            % yaw_e(i)   = (yaw_e(i-idz, :));
            % lat_e(i) = (lat_e(i-idz:i , :));
            % lon_e(i) = (lon_e(i-idz:i , :));
            % h_e(i)   = (h_e(i-idz:i , :));

            zupt_flag = true;

            % fprintf(' z\n')       % DEBUG
        end
    end

    %% KALMAN FILTER UPDATE

    % Check if there is a new vor_dme measurement to process at current INS time
    gdx =  find (vor_dme.t >= (imu.t(i) - vor_dme.eps) & vor_dme.t < (imu.t(i) + vor_dme.eps));

    if ( ~isempty(gdx) && gdx(1) > 1)

        gdx = gdx(1);
        %  gdx       % DEBUG

        %% MEASUREMENTS
        % 

        %    For Debug:
        % 
        % % VOR hatası hesaplaması
        % vor_error = vor_dme.distance(gdx) * tand(1);  % 1 derece VOR hatası
        % 
        % H_dme = 0.137 * (317 * (1 - exp(-0.137 * -vor_dme.h(gdx))) / 0.137 - vor_dme.h(gdx));
        % dme_error = H_dme * vor_dme.distance(gdx) * csc(deg2rad(vor_dme.bearing(gdx)) );
        % 
        % % Toplam hata standart sapması
        % total_error = sqrt(vor_error^2 + dme_error^2);
        % 
        % % Hataları vektörlere ekleme
        % vor_errors(i) = vor_error;
        % dme_errors(i) = dme_error;
        % total_errors(i) = total_error;

    
        % Meridian and normal radii of curvature update
        [RM,RN] = radius(lat_e(i));

        % Radians-to-meters matrix
        Tpr = diag([(RM + h_e(i)), (RN + h_e(i)) * cos(lat_e(i)), -1]);

        % Position innovations in meters with lever arm correction
        zp = Tpr * ([lat_e(i); lon_e(i); h_e(i);] - [vor_dme.lat(gdx); vor_dme.lon(gdx); vor_dme.h(gdx);]) ...
            + (DCMbn * vor_dme.larm);

        % Velocity innovations with lever arm correction
        zv = (vel_e(i,:) - vor_dme.vel(gdx,:) - ((omega_ie_n + omega_en_n) * (DCMbn * vor_dme.larm ))' ...
            + (DCMbn * skewm(wb_corrected) * vor_dme.larm )' )';

        %% KALMAN FILTER

        % vor_dme sampling interval
        dtg = vor_dme.t(gdx) - vor_dme.t(gdx-1);

        % Vector to update matrix F
        upd = [vel_e(i,:) lat_e(i) h_e(i) fn' wn'];

        % Matrices F and G update
        [kf.F, kf.G] = F_update(upd, DCMbn, imu);

        % Matrix H update
        if(zupt_flag == false)
            kf.H = [ O I O O O ;
                O O Tpr O O ; ];
            kf.R = diag([vor_dme.stdv vor_dme.stdm]).^2;
            kf.z = [ zv' zp' ]';
        else
            kf.H = [ O I O O O ; ];
            kf.R = diag([vor_dme.stdv]).^2;
            kf.z = zv;
        end

        % a posteriori states are forced to be zero (error-state approach)
        kf.xp = zeros(n , 1);
        % Execution of the extended Kalman filter
        kf = kalman(kf, dtg);

        %% OBSERVABILITY

        % Number the observable states at current vor_dme time
        ob(gdx) = rank(obsv(kf.F, kf.H));

        %% INS/vor_dme CORRECTIONS

        % Quaternion correction
        qua_skew = -skewm(qua(1:3));    % According to Crassidis, qua_skew should be
                                        % positive, but if positive NaveGo diverges.
        % Crassidis, Eq. A.174a
        Xi = [qua(4)*eye(3) + qua_skew; -qua(1:3)'];

        % Crassidis, Eq. 7.34
        qua = qua + 0.5 .* Xi * kf.xp(1:3);
        qua = qua / norm(qua);          % Brute-force normalization

        % DCM correction
        DCMbn = qua2dcm(qua);

        % Attitude correction, method 1
        %         euler = qua2euler(qua);
        %         roll_e(i) = euler(1);
        %         pitch_e(i)= euler(2);
        %         yaw_e(i)  = euler(3);

        % Attitude correction, method 2
        roll_e(i)  = roll_e(i)  - kf.xp(1);
        pitch_e(i) = pitch_e(i) - kf.xp(2);
        yaw_e(i)   = yaw_e(i)   - kf.xp(3);

        % Velocity correction
        vel_e(i,1) = vel_e(i,1) - kf.xp(4);
        vel_e(i,2) = vel_e(i,2) - kf.xp(5);
        vel_e(i,3) = vel_e(i,3) - kf.xp(6);

        % Position correction
        lat_e(i) = lat_e(i) - kf.xp(7);
        lon_e(i) = lon_e(i) - kf.xp(8);
        h_e(i)   = h_e(i)   - kf.xp(9);

        % Biases estimation
        gb_dyn   = -kf.xp(10:12);
        ab_dyn   = -kf.xp(13:15);

        % Matrices for later Kalman filter performance analysis
        xi(gdx,:) = kf.xi';
        xp(gdx,:) = kf.xp';
        b(gdx,:) = [gb_dyn', ab_dyn'];
        A(gdx,:)  = reshape(kf.A,  1, n^2);
        Pi(gdx,:) = reshape(kf.Pi, 1, n^2);
        Pp(gdx,:) = reshape(kf.Pp, 1, n^2);

        if(zupt_flag == false)
            v(gdx,:)  = kf.v';
            z(gdx,:)  = kf.z';
            K(gdx,:)  = reshape(kf.K, 1, n*r);
            S(gdx,:)  = reshape(kf.S, 1, r^2);
        else
            zupt_flag = false;
            z(gdx,:)  = [ kf.z' 0 0 0 ]';
            v(gdx,:)  = [ kf.v' 0 0 0 ]';
            K(gdx,1:n*3) = reshape(kf.K, 1, n*3);
            S(gdx,1:9)  = reshape(kf.S, 1, 3^2);
        end
    end
end

%% Summary from INS/vor_dme integration

nav_e.t     = imu.t(1:i, :);    % INS time vector
nav_e.tg    = vor_dme.t;           % vor_dme time vector, which is the time vector when the Kalman filter was executed
nav_e.roll  = roll_e(1:i, :);   % Roll
nav_e.pitch = pitch_e(1:i, :);  % Pitch
nav_e.yaw   = yaw_e(1:i, :);    % Yaw
nav_e.vel   = vel_e(1:i, :);    % NED velocities
nav_e.lat   = lat_e(1:i, :);    % Latitude
nav_e.lon   = lon_e(1:i, :);    % Longitude
nav_e.h     = h_e(1:i, :);      % Altitude
nav_e.gn    = gn_e(1:i, :);     % Gravity estimation in the nav-frame.

nav_e.xi    = xi;       % A priori states
nav_e.xp    = xp;       % A posteriori states
nav_e.z     = z;        % INS/vor_dme measurements
nav_e.v     = v;        % Kalman filter innovations
nav_e.b     = b;        % Biases compensations

nav_e.A     = A;        % Transition matrices
nav_e.Pi    = Pi;       % A priori covariance matrices
nav_e.Pp    = Pp;       % A posteriori covariance matrices
nav_e.K     = K;        % Kalman gain matrices
nav_e.S     = S;        % Innovation matrices
nav_e.ob    = ob;       % Number of observable states after each vor_dme data arriving

fprintf('\n');

end


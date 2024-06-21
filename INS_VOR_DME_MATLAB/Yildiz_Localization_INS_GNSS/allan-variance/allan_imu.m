function imu_allan = allan_imu (imu_sta)
% allan_imu: performs Allan variance analysis on inertial measurements
% coming from an IMU in order to characterize several types of IMU errors.
%
% INPUT
%   imu_sta, input data structure must contains the following fields:
%   
%   fb, Nx3 accelerations [X Y Z] (m/s^2).
%   wb, Nx3 turn rates [X Y Z] (rad/s).
%   t,  Nx1 time vector (s).
%
% OUTPUT
%   imu_allan, input data structure with the following addtional fields:
%
%   vrw, 1x3 velocity random walk (m/s^2/root(Hz)). 
%       Note: m/s^2/root(Hz) = m/(s^2*(1/s)^(1/2)) = m*(1/s)^(3/2) =
%           = m/s/root(s).
%
%   arw, 1x3 angle random walk (rad/s^2/root(Hz)). 
%       Note: rad/s/root(Hz) = rad/(s*(1/s)^(1/2)) = rad*(1/s)^(1/2) =
%           = rad/root(s).
%
%   vrrw, 1x3 velocity rate random walk (m/s^3/root(Hz)). 
%
%   arrw, 1x3 angle rate random walk (rad/s^2/root(Hz)). 
%
%   ab_dyn, 1x3 accrs bias instability (m/s^2). Value is taken
%       from the plot at the minimun value.
%
%   gb_dyn: 1x3 gyros bias instability (rad/s). Value is taken
%       from the plot at the minimun value.
%   
%   gb_corr, 1x3 gyros correlation times (s).
%   ab_corr, 1x3 accrs correlation times (s).
%
%   gb_psd, gyro dynamic bias root-PSD [X Y Z] (rad/s/root(Hz))
%   ab_psd, acc dynamic bias root-PSD [X Y Z] (m/s^2/root(Hz))
%
%

% Verbose for allan_overlap function
if (nargin < 2), verbose = 2; end

%% PREALLOCATION
%%

% Random walk
imu_allan.vrw = zeros(1,3);
imu_allan.arw = zeros(1,3);

% Rate random walk
imu_allan.vrrw = zeros(1,3);
imu_allan.arrw = zeros(1,3);

% Bias instability
imu_allan.ab_dyn = zeros(1,3);
imu_allan.gb_dyn = zeros(1,3);

% Bias instability correlation time
imu_allan.ab_corr = zeros(1,3);
imu_allan.gb_corr = zeros(1,3);

%% TIMESPAN

T = imu_sta.t(end) - imu_sta.t(1);

fprintf('allan_imu: time vector is %.2f hours or %.2f minutes or %.2f seconds long. \n', (T/60/60), (T/60), T)

%% ACCELEROMETERS
%%
text_st = {'ACCR X', 'ACCR Y', 'ACCR Z'};

for i=1:3
    
    fprintf('allan_imu: Allan variance for ACCR %d \n', i)   
    
%    plot_imu_sta(imu_sta.fb(:,i), imu_sta.freq, text_st(i));
    
    [N, K, B, tauB] = allan_matlab (imu_sta.fb(:,i), imu_sta.freq, text_st(i));
    
    imu_allan.vrw(i) = N;
    imu_allan.vrrw(i) = K;    
    imu_allan.ab_dyn(i) = B;
    imu_allan.ab_corr(i) = tauB;
    
end

%% GYROSCOPES
%%
text_st = {'GYRO X', 'GYRO Y', 'GYRO Z'};

for i=1:3
    
    fprintf('allan_imu: Allan variance for GYRO %d \n', i)   
    
%    plot_imu_sta(imu_sta.wb(:,i), imu_sta.freq, text_st(i));
    
    [N, K, B, tauB] = allan_matlab (imu_sta.wb(:,i), imu_sta.freq, text_st(i));
    
    imu_allan.arw(i) = N;
    imu_allan.arrw(i) = K;    
    imu_allan.gb_dyn(i) = B;
    imu_allan.gb_corr(i) = tauB;
end

%% Dynamic bias root-PSD
%%

imu_allan.ab_psd = imu_allan.ab_dyn .* sqrt(imu_allan.ab_corr) ;  % m/s^2/root(Hz)
imu_allan.gb_psd = imu_allan.gb_dyn .* sqrt(imu_allan.gb_corr) ;  % rad/s/root(Hz)

end

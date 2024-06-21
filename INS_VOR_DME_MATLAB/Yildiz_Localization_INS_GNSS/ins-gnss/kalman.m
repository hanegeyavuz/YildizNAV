function  kf = kalman(kf, dt)
% kalman: Kalman filter algorithm.
%
% INPUT
%   kf, data structure with at least the following fields:
%       xp: nx1 a posteriori state vector.
%        z: rx1 measurement vector.
%        F: nxn state transition matrix.
%        H: rxn observation matrix.
%        Q: qxq process noise covariance matrix.
%        R: rxr observation noise covariance matrix.
%       Pp: nxn a posteriori error covariance matrix.
%        G: nxq control-input matrix.      
%   	dt: sampling interval. 
%
% OUTPUT
%    kf, the following fields are updated:
%       xi: nx1 a priori state vector.
%       xp: nx1 a posteriori state vector.
%		 v: rx1 innovation vector. 
%        A: nxn state transition matrix.
%        K: nxr Kalman gain matrix.
%       Qd: nxn discrete process noise covariance matrix.
%       Pi: nxn a priori error covariance.
%       Pp: nxn a posteriori error covariance.  
%        S: rxr innovation (not residual) covariance matrix.
%

%% PREDICTION STEP

kf = kf_prediction(kf, dt);

%% UPDATE STEP

kf = kf_update(kf);

end

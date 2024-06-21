function  kf = kf_prediction(kf, dt)
% kf_prediction: prediction update part of the Kalman filter algorithm.
%
% INPUT
%   kf, data structure with at least the following fields:
%       xp: nx1 a posteriori state vector.
%       Pp: nxn a posteriori error covariance matrix.
%        F: nxn state transition matrix.
%        Q: qxq process noise covariance matrix.
%        G: nxq control-input matrix.      
%   	dt: sampling interval. 
%
% OUTPUT
%   kf, the following fields are updated:
%       xi: nx1 a priori state vector.
%       Pi: nxn a priori error covariance matrix.
%        A: nxn state transition matrix.
%       Qd: nxn discrete process noise covariance matrix.


% Discretization of continous-time system
kf.A =  expm(kf.F*dt);          				% Exact solution for linear systems
% kf.A = I + (kf.F * dt);         				% Approximated solution by Euler method 
kf.Qd = (kf.G * kf.Q * kf.G') .* dt;            % Digitalized covariance matrix

% Step 1, predict the a priori state vector, xi
kf.xi = kf.A * kf.xp;

% Step 2, update the a priori covariance matrix, Pi
kf.Pi = (kf.A * kf.Pp * kf.A') + kf.Qd;
kf.Pi =  0.5 .* (kf.Pi + kf.Pi');               % Force Pi to be a symmetric matrix

end

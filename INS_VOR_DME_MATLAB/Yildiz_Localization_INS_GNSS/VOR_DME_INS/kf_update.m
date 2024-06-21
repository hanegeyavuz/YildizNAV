function  kf = kf_update(kf)
% kf_update: measurement update part of the Kalman filter algorithm.
%
% INPUT
%   kf, data structure with at least the following fields:
%       xi: nx1 a priori state vector.
%       Pi: nxn a priori error covariance matrix.
%        z: rx1 measurement vector.
%        H: rxn observation matrix.
%        R: rxr observation noise covariance matrix.
%
% OUTPUT
%    kf, the following fields are updated:
%       xp: nx1 a posteriori state vector.
%       Pp: nxn a posteriori error covariance matrix.
%		 v: rx1 innovation vector.
%        K: nxr Kalman gain matrix.
%        S: rxr innovation (not residual) covariance matrix.


% I = eye(size(kf.F));

% Step 3, update Kalman gain
kf.S = (kf.R + kf.H * kf.Pi * kf.H');			% Innovation covariance matrix
kf.v =  kf.z - kf.H * kf.xi; 					% Innovation vector

r = length(kf.v);
if rank (kf.S) < r
    error('kf_update: innovation covariance matrix S is not invertable.')
end

kf.K = (kf.Pi * kf.H') * (kf.S)^(-1) ;			% Kalman gain matrix

% Step 4, update the a posteriori state vector, xp
kf.xp = kf.xi + kf.K * kf.v;

% Step 5, update the a posteriori covariance matrix, Pp
kf.Pp = kf.Pi - kf.K * kf.S * kf.K';            % Eq. 3.10 from Särkkä
% J = (I - S.K * S.H);                          % Joseph stabilized version
% S.Pp = J * S.Pi * J' + S.K * S.R * S.K';      % Alternative implementation
kf.Pp =  0.5 .* (kf.Pp + kf.Pp');               % Force Pp to be a symmetric matrix

end

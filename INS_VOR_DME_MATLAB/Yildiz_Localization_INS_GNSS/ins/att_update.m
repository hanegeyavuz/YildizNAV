function [qua, DCMbn, euler] = att_update(wb, DCMbn, qua, omega_ie_n, omega_en_n, dt, att_mode)
% att_update: updates attitude using quaternion or DCM.
%
% INPUT
%   wb,         3x1 incremental turn-rates in body-frame (rad/s).
%   DCMbn,      3x3 body-to-nav DCM.
%   qua,        4x1 quaternion.
%   omega_ie_n, 3x3 skew-symmetric Earth rate matrix (rad/s).
%   omega_en_n, 3x3 skew-symmetric transport rate (rad/s).
%   dt,         1x1 IMU sampling interval (s).
%	att_mode,   attitude mode (string).
%      'quaternion': attitude updated as quaternion. Default value.
%             'dcm': attitude updated as Direct Cosine Matrix.
%
% OUTPUT
%   qua,      4x1 updated quaternion.
%   DCMbn,    3x3 updated body-to-nav DCM.
%   euler,    3x1 updated Euler angles (rad).

if nargin < 7, att_mode  = 'quaternion'; end

%% Gyros output correction for Earth and transport rates

om_ie_n = skewm_inv(omega_ie_n);
om_en_n = skewm_inv(omega_en_n);
wb = (wb - DCMbn' * (om_ie_n + om_en_n));  % Titterton, Eq. 3.29, p. 32

if strcmp(att_mode, 'quaternion')
%% Quaternion update   

    qua   = qua_update(qua, wb, dt);    % Quaternion update
    qua   = qua / norm(qua);            % Brute-force normalization
    DCMbn = qua2dcm(qua);               % DCM update
    euler = qua2euler(qua);             % Euler angles update
    
elseif strcmp(att_mode, 'dcm')
%% DCM update    
    
    delta_theta = wb * dt;                  % Incremental Euler angles 
    DCMbn = dcm_update(DCMbn, delta_theta); % DCM update
    euler = dcm2euler(DCMbn);               % Euler angles update
    qua   = euler2qua(euler);               % Quaternion update
    qua   = qua / norm(qua);                % Brute-force normalization
    
else
    error('att_update: no attitude update mode defined. Check the attitude update mode selected.')
end

end

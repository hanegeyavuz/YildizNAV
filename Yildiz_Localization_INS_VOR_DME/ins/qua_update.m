function qua = qua_update(qua, wb, dt)
% qua_update: updates quaternions.
%
% INPUT
%   qua,    4x1 quaternion.
%   wb,     3x1 incremental turn rates in body-frame [X Y Z] (rad/s).
%   dt,     1x1 IMU sampling interval (s).
%
% OUTPUT
%   qua,    4x1 updated quaternion.

wnorm = norm(wb);

if wnorm < 1.E-8
    
    return;
else
    
    co = cos(double(0.5*wnorm*dt));
    si = sin(double(0.5*wnorm*dt));
    
    % Eq. 7.41
    psi = (si / wnorm) * wb;  
    
    % Eq. 7.40
    Om = [ (co*eye(3)-skewm(psi))  psi; % 3x4
           -psi'                   co]; % 1x4
    
    % Eq. 7.39
    qua = Om * qua;
end

end

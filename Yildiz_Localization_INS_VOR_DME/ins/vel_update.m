function vel_n = vel_update(fn, vel_o, omega_ie_n, omega_en_n, gn, dt)
% vel_update: updates velocity vector in the NED frame.
%
% INPUT
%   fn, 3x1 specific forces in the nav-frame (m/s^2).    
%   vel_o, 3x1 previous (old) velocity vector in the nav-frame (m/s). 
%   omega_ie_n, 3x3 previous skew-symmetric Earth rate matrix in the nav-frame (rad/s).
%   omega_en_n, 3x3 previous skew-symmetric transport rate matrix in the nav-frame (rad/s).
%   gn, 3x1 previous gravity in the nav-frame (m/s^2).
%   dt, 1x1 integration time step (s).
%
% OUTPUT
%   vel_n, 3x1 updated velocity vector in the nav-frame (m/s). 

    
 vel_o = double(vel_o);
 dt = double(dt);
 
coriolis = (omega_en_n + 2 * omega_ie_n);  % Coriolis 

fn_c = fn + gn - (coriolis * vel_o');      % Corrected specific force in nav-frame

fn_c = double(fn_c);

vel_n = vel_o + (fn_c' * dt) ;             % Velocity update

end

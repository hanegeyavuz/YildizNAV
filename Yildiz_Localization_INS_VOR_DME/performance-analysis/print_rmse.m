function rmse_v = print_rmse (ins_gnss, gnss, ref_n, ref_g, string)
% print_rmse: print on console Root Mean Squared Errors (RMSE) between INS/GNSS
% and a reference, and between GNSS-only and a reference as well.
%
% INPUT
%   ins_gnss, INS/GNSS data structure.
%   gnss, GNSS data structure.
%   ref_i, Reference data structure ajusted for INS/GNSS measurements.
%   ref_n, Reference data structure ajusted for GNSS measurements.
%   string, string to print on console identifying the INS/GNSS system.
%
% OUTPUT
%   rmse_v, vector with all RMSE.
%
%   rmse_v = [  RMSE_roll;  RMSE_pitch; RMSE_yaw;    
%               RMSE_vn;    RMSE_ve;    RMSE_vd;
%               RMSE_lat;   RMSE_lon;   RMSE_h;
%               RMSE_vn_g;  RMSE_ve_g;  RMSE_vd_g;
%               RMSE_lat_g; RMSE_lon_g; RMSE_h_g; ];



rmse_v = yildiz_rmse (ins_gnss, gnss, ref_n, ref_g);
        
%% Print RMSE

fprintf( '\nprint_rmse: RMSE for %s\n\n', string);
% We do not have any reference data
% fprintf(' Roll,  %s = %.4e deg \n',   string, rmse_v(1));
% fprintf(' Pitch, %s = %.4e deg \n',   string, rmse_v(2));   

% fprintf(' Yaw,   %s = %.4e deg \n\n', string, rmse_v(3));

if (isfield(ins_gnss, 'vel') && isfield( ref_n, 'vel') && isfield(gnss, 'vel') && isfield( ref_g, 'vel'))
    fprintf(' Vel. N, %s = %.4e m/s, GNSS = %.4e m/s \n',   string, rmse_v(4), rmse_v(10));
    fprintf(' Vel. E, %s = %.4e m/s, GNSS = %.4e m/s \n',   string, rmse_v(5), rmse_v(11));
    fprintf(' Vel. D, %s = %.4e m/s, GNSS = %.4e m/s \n\n', string, rmse_v(6), rmse_v(12));
end

fprintf(' Latitude,  %s = %.4e m, GNSS = %.4e m \n', string, rmse_v(7), rmse_v(13));
fprintf(' Longitude, %s = %.4e m, GNSS = %.4e m \n', string, rmse_v(8), rmse_v(14));
fprintf(' Altitude,  %s = %.4e m, GNSS = %.4e m \n', string, rmse_v(9), rmse_v(15));

end

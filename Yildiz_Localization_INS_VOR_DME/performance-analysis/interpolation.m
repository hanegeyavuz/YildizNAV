function [nav_i, ref] = interpolation (nav, ref)
% interpolation: interpolates navigation data using a reference time
% vector.
%
% INPUT
%   nav, navigation data structure to be interpolated.
%   ref, reference data structure.
%
% OUTPUT
%   nav_i, navigation data structure interpolated by reference time vector.
%   ref,   reference data structure adjusted by the nav time vector.

nav.t = double(nav.t);
nav.lat = double(nav.lat);
nav.lon = double(nav.lon);
nav.h = double(nav.h);
nav.vel = double(nav.vel); % Eğer varsa

ref.t = double(ref.t);
ref.lat = double(ref.lat);
ref.lon = double(ref.lon);
ref.h = double(ref.h);

D = max(size(nav.t));
R = max(size(ref.t));

if (D > R)
    
    method = 'nearest';
else
    
    method = 'spline';
end

%% Adjust reference data structure before interpolating

if (ref.t(1) < nav.t(1))
    
    fprintf('Yildiz_NAV_interpolation: adjusting first element of ref ... \n')
    
    idx  = find(ref.t >= nav.t(1), 1, 'first' );
    if(isempty(idx))
        error('Yildiz_NAV_interpolation: idx empty index.')
    end
    
    ref.t   = ref.t  (idx:end);
    ref.lat = ref.lat(idx:end);
    ref.lon = ref.lon(idx:end);
    ref.h   = ref.h(  idx:end);
    
    if (isfield(ref, 'vel'))
        ref.vel = ref.vel(idx:end, :);
    end
    
    if (isfield(ref, 'roll'))
        ref.roll  = ref.roll (idx:end);
        ref.pitch = ref.pitch(idx:end);
        ref.yaw   = ref.yaw  (idx:end);
    end
end

if (ref.t(end) > nav.t(end))
    
    fprintf('Yildiz_NAV_interpolation: adjusting last element of ref... \n')
    
    idx  = 1;
    fdx  = find(ref.t <= nav.t(end), 1, 'last' );
    if(isempty(fdx))
        error('Yildiz_NAV_interpolation: fdx empty index.')
    end
    
    ref.t   = ref.t  (idx:fdx);
    ref.lat = ref.lat(idx:fdx);
    ref.lon = ref.lon(idx:fdx);
    ref.h   = ref.h  (idx:fdx);
    
    if (isfield( ref, 'vel'))
        ref.vel = ref.vel(idx:fdx, :);
    end
    
    if (isfield(ref, 'roll'))
        ref.roll  = ref.roll (idx:fdx);
        ref.pitch = ref.pitch(idx:fdx);
        ref.yaw   = ref.yaw  (idx:fdx);
    end
end

%% Interpolation

% If data is from INS/VOR/DME solution...

if (isfield(nav, 'roll') && isfield(ref, 'roll'))
    
    fprintf('Yildiz_NAV_interpolation: %s method to interpolate INS/VOR/DME solution\n', method)
    

    %  [nav_unique_t, unique_indices] = unique(nav.t);
    %  nav_unique_pitch = nav.pitch(unique_indices);
    % 
    % 
    % nav_i.t     = ref.t;
    % nav_i.roll  = interp1q(nav.t, nav.roll,  ref.t);
    % %nav_i.pitch = interp1(nav.t, (nav.pitch), ref.t);
    % nav_i.pitch = interp1(nav_unique_t, nav_unique_pitch, ref.t);
    % nav_i.yaw   = interp1(nav.t, (nav.yaw),   ref.t);
    % nav_i.lat   = interp1(nav.t, (nav.lat),   ref.t);
    % nav_i.lon   = interp1(nav.t, (nav.lon),   ref.t);
    % nav_i.h     = interp1(nav.t, (nav.h),     ref.t);


    % Benzersiz örnek noktaları ve bunlara karşılık gelen benzersiz değerleri alın
    [nav_unique_t, unique_indices] = unique(nav.t);
    %nav_unique_roll = nav.roll(unique_indices);
    %nav_unique_pitch = nav.pitch(unique_indices);
    %nav_unique_yaw = nav.yaw(unique_indices);
    nav_unique_lat = nav.lat(unique_indices);
    nav_unique_lon = nav.lon(unique_indices);
    nav_unique_h = nav.h(unique_indices);
    
    % Yeniden örnekleme işlemini gerçekleştirin
    nav_i.t     = ref.t;
    nav_i.roll  = nav.roll;%interp1(nav_unique_t, nav_unique_roll, ref.t);
    nav_i.pitch = nav.pitch;%interp1(nav_unique_t, nav_unique_pitch, ref.t);
    nav_i.yaw   = nav.yaw;%interp1(nav_unique_t, nav_unique_yaw, ref.t);
    nav_i.lat   = interp1(nav_unique_t, nav_unique_lat, ref.t);
    nav_i.lon   = interp1(nav_unique_t, nav_unique_lon, ref.t);
    nav_i.h     = interp1(nav_unique_t, nav_unique_h, ref.t);



    
    if (isfield(ref, 'vel') && isfield( nav, 'vel'))
        
        nav_i.vel = interp1(nav.t, unique(nav.vel),   ref.t, method);
        flag_vel  = any(isnan(nav_i.vel));
    else
        flag_vel = false(1,3);
    end
    
    flag = any(isnan(nav_i.t)) | any(isnan(nav_i.roll)) | any(isnan(nav_i.pitch)) | any(isnan(nav_i.yaw)) |  ...
        any(isnan(nav_i.lat)) | any(isnan(nav_i.lon)) | any(isnan(nav_i.h)) | flag_vel;
    
    % Test interpolated dataset
    if(flag)
        
        error('Yildiz_NAV_interpolation: NaN value in INS/VOR/DME interpolated solution')
    end
    
% If dataset is from a GNSS-only solution...

else
    
    fprintf('Yildiz_NAV_interpolation: %s method to interpolate GNSS-only solution\n', method)

    [nav_unique_t_o, unique_indices_o] = unique(nav.t);
    %nav_unique_vel_o = unique(nav.vel);
    nav_unique_lat_o = nav.lat(unique_indices_o);
    nav_unique_lon_o = nav.lon(unique_indices_o);
    nav_unique_h_o = nav.h(unique_indices_o);
    nav_unique_vel_o = nav.vel(unique_indices_o);
    
   
    nav_i.t   = ref.t;
    nav_i.lat = interp1(nav_unique_t_o, nav_unique_lat_o, ref.t, method);        
    nav_i.vel = interp1(nav_unique_t_o, nav_unique_vel_o,   ref.t, method);
    nav_i.lon = interp1(nav_unique_t_o, nav_unique_lon_o, ref.t, method);
    nav_i.h   = interp1(nav_unique_t_o, nav_unique_h_o,   ref.t, method);


    
    if (isfield(ref, 'vel') && isfield( nav, 'vel'))
        
        nav_i.vel = interp1(nav_unique_t_o, nav_unique_vel_o,   ref.t, method);
        flag_vel = any(isnan(nav_i.vel));
    else
        flag_vel = false(1,3);
    end

    
    flag = any(isnan(nav_i.t)) | any(isnan(nav_i.lat)) | any(isnan(nav_i.lon)) | ...
        any(isnan(nav_i.h)) | flag_vel;
    
    % Test interpolated dataset
    if(flag)
        
        error('Yildiz_NAV_interpolation: NaN value in GNSS-only interpolated solution')
    end
end

end

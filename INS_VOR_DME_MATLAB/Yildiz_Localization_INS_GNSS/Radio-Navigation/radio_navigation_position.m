function [LAT,LON] = radio_navigation_position(vor_dm,stat)

    % Dünya yarıçapı (kilometre cinsinden)
    R = 6371;

    D2R = (pi/180);     % Degrees to radians
    R2D = (180/pi);     % Radians to degrees


    % Enlem ve boylamı radyan cinsine çevir
    vor_dm.lat = (stat.lat)* D2R;
    vor_dm.lon = (stat.lon)* D2R;
    bearing_rad = (vor_dm.bearing)* D2R;
    
    % DME mesafesini kilometreye çevir
    dme_distance_km = vor_dm.distance / 1000; % 1 deniz mili = 1.852 km

    % Hedef noktanın koordinatlarını hesapla
    target_lat_rad = asin(sin(vor_dm.lat) .* cos(dme_distance_km / R) + cos(vor_dm.lat) .* sin(dme_distance_km / R) .* cos(bearing_rad));
    target_lon_rad = vor_dm.lon + atan2(sin(bearing_rad) .* sin(dme_distance_km / R) .* cos(vor_dm.lat),cos(dme_distance_km / R) - sin(vor_dm.lat) .* sin(target_lat_rad));

    % Hedef noktanın koordinatlarını derece cinsine çevir
    LAT = (target_lat_rad) * R2D;
    LON = (target_lon_rad) * R2D;
end


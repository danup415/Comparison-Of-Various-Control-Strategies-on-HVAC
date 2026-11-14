function [alt, azi] = solar_geometry(lat, lon, tz, doy, hour_mid)
%SOLAR_GEOMETRY  Solar altitude and azimuth (deg; azimuth clockwise from north).
%   doy: day of year (1..365); hour_mid: local standard clock time (h, e.g. 12.5).
%   Spencer (1971) declination and equation of time.
B    = 2*pi*(doy - 1)/365;
decl = 0.006918 - 0.399912*cos(B) + 0.070257*sin(B) - 0.006758*cos(2*B) ...
     + 0.000907*sin(2*B) - 0.002697*cos(3*B) + 0.00148*sin(3*B);          % rad
eot  = 229.18*(0.000075 + 0.001868*cos(B) - 0.032077*sin(B) ...
     - 0.014615*cos(2*B) - 0.040849*sin(2*B));                            % min
tsol = hour_mid + (4*(lon - 15*tz) + eot)/60;                             % h
ha   = deg2rad(15*(tsol - 12));
phi  = deg2rad(lat);
sinalt = sin(phi).*sin(decl) + cos(phi).*cos(decl).*cos(ha);
alt  = asin(max(-1, min(1, sinalt)));
cosaz = (sin(decl) - sin(alt).*sin(phi)) ./ max(1e-9, cos(alt).*cos(phi));
az   = acos(max(-1, min(1, cosaz)));
az(ha > 0) = 2*pi - az(ha > 0);                                           % afternoon -> west
alt = rad2deg(alt); azi = rad2deg(az);
end

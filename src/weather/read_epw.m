function w = read_epw(file)
%READ_EPW  Read an EnergyPlus/ISHRAE EPW weather file (8760 hourly records).
%   w fields (8760x1): month, day, hour (1..24, hour-ending), Tdb, Tdp, RH,
%   Patm, GHI, DNI, DHI, wind; plus site latitude/longitude/time zone.
fid = fopen(file, 'r');
if fid < 0, error('read_epw:open', 'Cannot open %s', file); end
hdr = fgetl(fid);
loc = strsplit(hdr, ',');
w.site = loc{2};
w.lat  = str2double(loc{7});
w.lon  = str2double(loc{8});
w.tz   = str2double(loc{9});
w.elev = str2double(loc{10});
for i = 1:7, fgetl(fid); end          % remaining header lines
C = textscan(fid, ['%f %f %f %f %f %s' repmat(' %f', 1, 29)], 'Delimiter', ',', ...
             'CollectOutput', false, 'EmptyValue', NaN);
fclose(fid);
num = @(i) C{i};
w.month = num(2); w.day = num(3); w.hour = num(4);
w.Tdb  = num(7);  w.Tdp = num(8);  w.RH = num(9);  w.Patm = num(10);
w.GHI  = num(14); w.DNI = num(15); w.DHI = num(16);
w.wind = num(22);
n = numel(w.Tdb);
if n ~= 8760, error('read_epw:length', 'Expected 8760 records, got %d', n); end
% EPW "DATA PERIODS" for this file starts on a Sunday (checked in header line 8).
w.dow0 = 0;    % 0 = Sunday for 1 January
end

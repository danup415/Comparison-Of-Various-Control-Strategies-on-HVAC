function D = build_inputs(cfg, w)
%BUILD_INPUTS  All exogenous time series at the plant step cfg.Ts (N = days*86400/Ts).
%   Weather is taken from the EPW file w (hour-ending records). Temperatures and
%   humidity ratio are linearly interpolated to step mid-points; irradiance is
%   held constant over the hour it was recorded in.
b = cfg.bld; o = cfg.occ; q = cfg.iaq;
spd = 86400/cfg.Ts;                       % steps per day
N   = cfg.days * spd;
k0  = (cfg.start_day - 1) * spd;
t_mid = ((k0 + (1:N)') - 0.5) * cfg.Ts / 3600;     % hours since 1 Jan 00:00 (step mid-point)
hr_idx = min(8760, floor(t_mid) + 1);              % EPW record covering this step (hour-ending)

% --- interpolate temperatures / moisture (EPW value at hour-ending time h)
th  = (1:8760)';
Tdb = w.Tdb + cfg.weather_offset_K;
Wout_h = psy('w', w.Tdb, w.RH, w.Patm);            % keep absolute humidity under a heatwave shift
ext = @(v) interp1([0; th; 8761], [v(end); v; v(1)], t_mid, 'linear');
D.To   = ext(Tdb);
D.Wout = ext(Wout_h);
D.Patm = ext(w.Patm);
D.RHout = psy('rh', D.To, D.Wout, D.Patm);
D.Twb  = psy('twb', D.To, D.RHout);
GHI = w.GHI(hr_idx); DNI = w.DNI(hr_idx); DHI = w.DHI(hr_idx);
D.GHI = GHI;

% --- calendar
doy   = floor((t_mid)/24) + 1;
hod   = floor(mod(t_mid, 24));                     % 0..23
D.doy = doy; D.hod = hod;
D.month = w.month(hr_idx);
dow   = mod(w.dow0 + doy - 1, 7);                  % 0 = Sunday
D.weekend = (dow == 0) | (dow == 6);

% --- solar on facades (sun position at the mid-point of the EPW hour)
hour_mid = mod(hr_idx - 1, 24) + 0.5;
[alt, azi] = solar_geometry(w.lat, w.lon, w.tz, floor((hr_idx-1)/24)+1, hour_mid);
sun_up = alt > 0;
rho_g = 0.20;
Isurf = zeros(N, 4);
for f = 1:4
    cth = cosd(alt) .* cosd(azi - b.azimuth(f));               % vertical surface incidence
    beam = DNI .* max(0, cth) .* sun_up .* (1 - b.shade_month(D.month)');
    Isurf(:,f) = beam + 0.5*DHI + 0.5*rho_g*GHI;
end
D.Isurf = Isurf;

% --- sol-air temperatures
D.Tsa_roof = D.To + (b.alpha_roof*GHI - b.eps_lw*b.dR) / b.h_out;
D.Tsa_wall = repmat(D.To, 1, 3) + b.alpha_wall * (Isurf * b.facade') / b.h_out;

% --- transmitted solar per zone (W)
A_glz = b.WWR * b.ext_perim * b.height;
D.Qsol = (Isurf * b.facade') .* repmat(A_glz * (1 - b.frame_frac) * b.SHGC, N, 1);

% --- occupancy, internal gains (W sensible per zone), latent (kg/s), CO2 generation
occ = zeros(N,3);
for z = 1:3
    occ(~D.weekend, z) = o.occ_weekday(hod(~D.weekend)+1, z);
    occ( D.weekend, z) = o.occ_weekend(hod( D.weekend)+1, z);
end
D.occ = occ;
light = [0.1 0.1 0.1 0.1 0.1 0.3 0.6 0.3 0.1 0.1 0.1 0.2 0.3 0.2 0.1 0.1 0.2 0.5 0.9 0.9 0.8 0.5 0.3 0.2];
equip = [0.2 0.1 0.1 0.1 0.1 0.2 0.5 0.7 0.6 0.3 0.2 0.5 0.6 0.3 0.2 0.3 0.4 0.7 0.9 0.9 0.8 0.6 0.4 0.3];
cook  = ismember(hod, o.cook_hours);
D.cook = cook;
Qint = occ*o.q_sens_W + (light(hod+1)'*o.light_Wm2 + equip(hod+1)'*o.equip_Wm2) * b.area;
Qint(:,2) = Qint(:,2) + cook*o.cook_sens_W*o.cook_to_Z2;
D.Qint = Qint;
D.Glat = (sum(occ,2)*o.q_lat_W + cook*o.cook_lat_W*o.cook_to_Z2) / 2.45e6;   % kg/s
D.npeople = sum(occ,2);

% --- outdoor PM2.5 (representative profile; replace with measured CPCB data)
PMo = q.PM_out_month(D.month)' .* q.PM_out_diurnal(hod+1)';
wash = (D.RHout > 85) & (GHI < 50);
PMo(wash) = 0.4 * PMo(wash);
D.PMout = PMo;

% --- tariff, comfort schedule
D.tariff = cfg.tariff(hod+1)';
c = cfg.comfort;
occd = occ > 0;
D.occupied = occd;
D.sp  = c.sp_unocc*ones(N,3);  D.sp(occd) = c.sp_occ;
D.Tlo = c.band_unocc(1)*ones(N,3); D.Tlo(occd) = c.band_occ(1);
D.Thi = c.band_unocc(2)*ones(N,3); D.Thi(occd) = c.band_occ(2);
D.N = N; D.spd = spd; D.Ts = cfg.Ts;
end

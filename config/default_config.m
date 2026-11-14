function cfg = default_config()
%DEFAULT_CONFIG  Single source of truth for every parameter in the study.
%   All numbers used anywhere in the code come from this struct. Values
%   marked [ASSUMED] are modelling assumptions that should be stated as such
%   in the manuscript; values marked [SOURCE] need the cited reference.

%% ---------------- Simulation ----------------
cfg.Ts        = 900;            % s, plant + low-level control step (15 min)
cfg.Ts_mpc    = 3600;           % s, MPC / supervisory update period
cfg.days      = 365;            % simulated days (1..365)
cfg.start_day = 1;              % day of year to start
cfg.seeds     = 1:5;            % replicate seeds (sensor noise, forecast noise, RL training)
cfg.sensor_noise_std = 0.10;    % K, zone air-temperature sensor noise [ASSUMED]

%% ---------------- Weather ----------------
cfg.epw_file  = fullfile('data','IND_Pune.430630_ISHRAE.epw');   % ISHRAE TMY, Pune 430630
cfg.weather_offset_K = 0;       % uniform dry-bulb shift (heatwave scenario uses +3 K)

%% ---------------- Building: 95 m2 top-floor 3BHK flat, 3 conditioned zones ----------------
b.names     = {'Z1 Master bedroom','Z2 Living+dining','Z3 Second bedroom'};
b.area      = [30 45 20];       % m2
b.height    = 2.85;             % m
b.ext_perim = [11 15 9.2];      % m of exterior wall per zone
% facade orientation share of each zone's exterior wall (columns N E S W)
b.facade    = [0.0 0.0 0.5 0.5;     % Z1 south + west
               0.4 0.0 0.0 0.6;     % Z2 north + west
               0.5 0.5 0.0 0.0];    % Z3 north + east
b.azimuth   = [0 90 180 270];       % deg from north, facade outward normal (N E S W)
b.WWR       = 0.20;             % window-to-wall ratio
b.U_wall    = 0.75;             % W/m2K, 200 mm AAC + 12 mm plaster both sides [ASSUMED, typical Pune new-build]
b.U_roof    = 1.20;             % W/m2K, 150 mm RCC + insulation/weathering course (Eco-Niwas Samhita roof limit) [SOURCE: ENS 2018]
b.U_glz     = 2.8;              % W/m2K, uPVC double glazing (kept from original study)
b.SHGC      = 0.55;
b.frame_frac= 0.25;             % opaque frame fraction of window area
b.shade_month = [0.15 0.18 0.25 0.35 0.40 0.45 0.45 0.42 0.38 0.28 0.18 0.12]; % chajja shading fraction of direct beam
b.alpha_wall = 0.40;            % solar absorptance, light-coloured plaster
b.alpha_roof = 0.70;            % solar absorptance, roof
b.eps_lw     = 0.90;  b.dR = 63;  % long-wave sky correction (W/m2) for horizontal surfaces (ASHRAE Fundamentals)
b.h_out      = 22.7;            % W/m2K exterior film
b.ACH_inf    = 0.50;            % 1/h infiltration [ASSUMED]
% thermal mass (per m2 of zone floor area unless stated), effective for the diurnal cycle
b.kappa_roof  = 150e3;          % J/m2K effective inner part of 150 mm RCC roof slab (2400*880*0.15 = 317 kJ/m2K total)
b.kappa_floor = 150e3;          % J/m2K effective upper part of 150 mm RCC floor slab
b.kappa_wall  = 55e3;           % J/m2K per m2 of opaque exterior wall (inner half of 200 mm AAC, 550 kg/m3)
b.kappa_furn  = 15e3;           % J/m2K furniture, lumped into the air node
b.h_in        = 3.0;            % W/m2K interior convective coefficient (air node <-> mass node)
b.R_part_m2   = 0.8786;         % m2K/W internal partition (115 mm brick + plaster + films)
b.part_len    = [6.0 5.7];      % m, partition length Z1-Z2 and Z2-Z3
b.solar_to_air= 0.30;           % fraction of transmitted solar to air node (rest to mass)
b.int_conv    = 0.60;           % convective fraction of internal gains
cfg.bld = b;

%% ---------------- Occupancy & internal gains ----------------
o.n_people  = 4;
o.q_sens_W  = 75;   o.q_lat_W = 55;       % per person, seated/sleeping [SOURCE: ASHRAE Fundamentals]
o.light_Wm2 = 5;    o.equip_Wm2 = 4.5;
o.cook_sens_W = 2000; o.cook_lat_W = 500; o.cook_to_Z2 = 0.30;  % open-kitchen leakage into living zone
o.cook_hours = [7 8 19 20];               % hour-of-day (0..23) when cooking occurs
% occupants per zone by hour (0..23); weekday / weekend
wd = zeros(24,3); we = zeros(24,3);
wd([1:7 23 24],1) = 2;  wd([1:7 23 24],3) = 2;          % bedrooms at night
wd([8 9 18:22],2) = 4;  wd(10:17,2) = 1;                 % living: mornings, evenings, one person home by day
we([1:8 23 24],1) = 2;  we([1:8 23 24],3) = 2;
we(9:22,2) = 3; we(15:16,1) = 1; we(15:16,2) = 2;        % weekend afternoon nap
o.occ_weekday = wd; o.occ_weekend = we;
cfg.occ = o;

%% ---------------- Comfort ----------------
c.band_occ   = [23 26];        % deg C, NBC 2005 summer range for air-conditioned buildings (NBC 2016 moved to an adaptive model)
c.band_unocc = [16 29];        % deg C, setup band when zone unoccupied
c.sp_occ     = 24.5;           % deg C, thermostat/PID setpoint when occupied (band midpoint)
c.sp_unocc   = 28.5;           % deg C, setup setpoint when unoccupied (same for all controllers)
c.RH_max     = 70;             % %, humidity discomfort threshold
cfg.comfort = c;

%% ---------------- Equipment ----------------
v.Qcap_rated  = 11.0;          % kW total cooling, outdoor unit at 35 C
v.Qiu_max     = [3.5 5.0 2.5]; % kW total cooling per indoor unit
v.COP_rated   = 3.8;           % at 35 C outdoor [ASSUMED, inverter VRF/multisplit]
v.cap_slope   = 0.008;         % fractional capacity loss per K above 35 C
v.eirT_slope  = 0.025;         % fractional EIR increase per K above 35 C
v.eirPLR      = [0.08 0.62 0.30]; % EIR(PLR)=a+b*PLR+c*PLR^2, generic inverter curve [ASSUMED]
v.PLR_min     = 0.20;  v.Cd = 0.25;   % min modulation, cycling degradation
v.SHR         = 0.75;  v.W_coil = 0.0087;  % sensible heat ratio, coil saturation humidity ratio (~12 C ADP)
v.P_iu_fan    = 0.030;         % kW per running indoor unit
v.T_oper      = [10 50];       % C outdoor operating range
cfg.vrf = v;

e.flow   = 0.35;  e.eff_wb = 0.80;  e.P_fan = 0.18;   % m3/s, wet-bulb effectiveness, kW fan+pump
e.T_on   = 28;    e.RH_on  = 50;    e.dT_min = 2.0;   % enable rules
e.filter_eta = 0.20;          % PM2.5 removal on IEC intake (coarse filter)
cfg.iec = e;

d.mode   = 'continuous';      % 'continuous' | 'humidity' | 'off'
d.flow   = 0.030;             % m3/s, ASHRAE 62.2-2022: 0.15*95 + 3.5*(3+1) = 28 L/s
d.flow_humid = 0.06;          % m3/s, used only in 'humidity' mode (original study)
d.erv_sens = 0.70;  d.erv_lat = 0.60;
d.W_sup_max = 0.0095;  d.W_in_on = 0.0115;   % coil enable: indoor W > 0.0115 (~58 %RH at 25 C)
  d.T_sup_coil = 15;  d.COP_coil = 3.0;  d.P_fan = 0.040;
d.filter_eta = 0.60;          % MERV-13 on outdoor air (PM2.5)
cfg.doas = d;

%% ---------------- Indoor air quality ----------------
q.CO2_out    = 420;           % ppm
q.CO2_gen    = 5.2e-6;        % m3/s per person [SOURCE: ASHRAE 62.1 App. D]
q.PM_pen     = 0.80;          % infiltration penetration factor [SOURCE: residential IAQ literature]
q.PM_dep     = 0.5/3600;      % 1/s deposition
q.PM_cook    = 20;            % ug/s emitted into conditioned volume while cooking (~1.2 mg/min after hood capture) [ASSUMED]
q.PM_bg      = 0.38;          % ug/s background indoor source
q.recirc_flow= 0.30;          % m3/s through recirculation filter
q.recirc_eta = 0.9995;        % HEPA H13 single-pass
q.recirc_on  = true;
q.PM_out_month = [55 48 42 35 30 18 12 10 14 30 50 60];  % ug/m3 monthly mean outdoor PM2.5 [REPLACE with CPCB CAAQMS hourly data]
q.PM_out_diurnal = [0.75 0.70 0.68 0.68 0.72 0.85 1.20 1.35 1.25 1.05 0.90 0.85 ...
                    0.85 0.85 0.88 0.92 1.00 1.10 1.35 1.40 1.25 1.10 0.95 0.82];
cfg.iaq = q;

%% ---------------- Economics ----------------
% HYPOTHETICAL three-tier ToD tariff by hour of day 0..23 (INR/kWh), kept from the original study.
% It is NOT the MSEDCL residential tariff; see scenario 'tariff_msedcl' for the actual structure
% (flat energy charge with a solar-hours rebate, MERC MYT order FY2025-26, Case 217 of 2024).
tod = 7.15*ones(1,24); tod(1:6) = 5.20; tod(24) = 5.20; tod(8:10) = 8.95; tod(19:23) = 8.95;
cfg.tariff = tod;
cfg.grid_factor = 0.727;       % kgCO2/kWh, CEA CO2 Baseline Database v20.0, Indian grid weighted average FY2023-24

%% ---------------- Plant/model mismatch (realism for model-based controllers) ----------------
mm.enable  = true;             % plant parameters differ from the controller's nominal model
mm.Cm      = 0.85;  mm.Ca = 1.10;  mm.UA = 1.10;  mm.solar = 1.10;  mm.int = 1.15;
cfg.mismatch = mm;

%% ---------------- Controllers ----------------
k.thermo.hyst = 0.5;           % K half-band

k.pid.tau_c   = 900;           % s, SIMC closed-loop time constant
k.pid.Td_frac = 0.0;           % derivative off by default (thermal loops); set >0 for PID
k.pid.Tt_frac = 1.0;           % back-calculation tracking time / Ti

k.mpc.N_h     = 24;            % horizon in MPC steps (hours)
k.mpc.rho     = 50;            % INR per K^2 per 15-min step, soft comfort penalty (same weight as DQN reward)
k.mpc.backoff = 0.5;           % K constraint tightening inside the comfort band (robustness margin)
k.mpc.lambda  = 0.05;          % INR per kW^2 move penalty
k.mpc.iters   = 120;           % FISTA iterations
k.mpc.fc_sigma= 0.15;          % K * sqrt(hours ahead), dry-bulb forecast error growth
k.mpc.fc_max  = 2.0;           % K cap
k.mpc.sol_err = 0.10;          % relative solar forecast error (1-sigma)
k.mpc.perfect = false;         % true: perfect model + perfect forecast (upper bound)

k.rl.levels   = [0 0.25 0.5 0.75 1.0];  % fraction of indoor-unit sensible capacity
k.rl.hidden   = [64 64];
k.rl.gamma    = 0.99;   k.rl.lr = 3e-4;  k.rl.batch = 128;  k.rl.buffer = 100000;
k.rl.episodes = 200;    k.rl.ep_days = 14;  k.rl.update_every = 2;  k.rl.tau = 0.005;
k.rl.shield = true;  k.rl.shield_pen = 5;  k.rl.grad_clip = 1.0;  k.rl.eps0 = 1.0;    k.rl.eps_min = 0.05;  k.rl.eps_frac = 0.6;
k.rl.beta     = 50;  k.rl.r_scale = 100;    k.rl.switch_pen = 0.02;   % reward weights (INR-equivalent per K^2, per switch)
k.rl.val_every= 10;     k.rl.val_starts = [20 100 160 240 320];   % validation fortnight start days (perturbed weather)
k.rl.perturb  = struct('dT',1.5,'solar',0.15,'int',0.15);

k.hyb.Kp_frac = 0.5;           % fraction of SIMC gain used for MPC-tracking correction
k.hyb.safety_dT = 0.5;         % K above occupied upper band that triggers safety override
k.hyb.safety_frac = 0.8;
cfg.ctrl = k;

cfg.controllers = {'thermostat','pid','mpc','rl','hybrid'};
cfg.labels      = {'Thermostat','PID','E-MPC','DQN','Hybrid'};
cfg.out_dir     = 'results';
end

function cfg = scenario_config(name)
%SCENARIO_CONFIG  Configuration for each experiment (all start from default_config).
%   base          main results (plant/model mismatch + forecast error + sensor noise)
%   heatwave      dry-bulb +3 K all year (unseen by the DQN during training)
%   mass_low/high effective thermal mass x0.5 / x2 (plant AND nominal model)
%   flat_tariff   flat INR 7.15/kWh (tests whether E-MPC savings come from ToD shifting)
%   mpc_perfect   perfect model + perfect forecast (upper bound for E-MPC/Hybrid)
%   no_mismatch   plant = nominal model (forecast/sensor errors remain)
%   horizon6/12   E-MPC prediction horizon 6 h / 12 h
%   rl_levels3/9  DQN action resolution 3 / 9 levels
%   rl_noshield   DQN trained and deployed without the safety shield
%   iaq_nofilter  no recirculation filtration
%   vent_humidity ventilation as in the original study (DOAS 60 L/s only when RH>65 %, T>22 C)
%   vent_off      no mechanical ventilation (infiltration only)
cfg = default_config();
switch name
    case 'base'
    case 'heatwave',     cfg.weather_offset_K = 3;
    case 'mass_low',     f = 0.5; cfg.bld.kappa_roof = f*cfg.bld.kappa_roof; cfg.bld.kappa_floor = f*cfg.bld.kappa_floor; cfg.bld.kappa_wall = f*cfg.bld.kappa_wall;
    case 'mass_high',    f = 2.0; cfg.bld.kappa_roof = f*cfg.bld.kappa_roof; cfg.bld.kappa_floor = f*cfg.bld.kappa_floor; cfg.bld.kappa_wall = f*cfg.bld.kappa_wall;
    case 'flat_tariff',  cfg.tariff = 7.15*ones(1,24);
    case 'mpc_perfect',  cfg.ctrl.mpc.perfect = true;
    case 'no_mismatch',  cfg.mismatch.enable = false;
    case 'horizon6',     cfg.ctrl.mpc.N_h = 6;
    case 'horizon12',    cfg.ctrl.mpc.N_h = 12;
    case 'rl_levels3',   cfg.ctrl.rl.levels = [0 0.5 1];
    case 'rl_levels9',   cfg.ctrl.rl.levels = linspace(0, 1, 9);
    case 'rl_noshield',  cfg.ctrl.rl.shield = false;
    case 'iaq_nofilter', cfg.iaq.recirc_on = false;
    case 'vent_humidity',cfg.doas.mode = 'humidity';
    case 'vent_off',     cfg.doas.mode = 'off';
    % ---- verification experiments for the explanations in the Discussion ----
    case 'heat_perfect',  cfg.weather_offset_K = 3; cfg.ctrl.mpc.perfect = true;   % heatwave, perfect model+forecast
    case 'heat_backoff1', cfg.weather_offset_K = 3; cfg.ctrl.mpc.backoff = 1.0;    % heatwave, larger comfort margin
    case 'tariff_msedcl', tod = 7.15*ones(1,24); tod(10:17) = 7.15 - 0.80; cfg.tariff = tod;  % solar-hours (09-17) rebate INR 0.80/kWh
    case 'tariff_steep',  tod = 7.15*ones(1,24); tod([1:6 24]) = 4.00; tod([8:10 19:23]) = 14.30; cfg.tariff = tod;  % 3.6:1
    case 'hyb_noPI',      cfg.ctrl.hyb.Kp_frac = 0;        % Hybrid without the PI correction layer
    case 'hyb_nosafety',  cfg.ctrl.hyb.safety_frac = 0;    % Hybrid without the safety override
    otherwise
        if strncmp(name, 'uq', 2), cfg = uq_config(str2double(name(3:end))); return; end
        error('scenario_config:name', 'Unknown scenario %s', name);
end
cfg.scenario = name;
end

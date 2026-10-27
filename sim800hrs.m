%% ================================================================
%% Pune_HVAC_Full_Simulink.m
%% COMPLETE SIMULINK MODEL GENERATOR — Pune 3BHK HVAC System
%% Covers: Days 1-10 (TMY → Building → Thermal → HVAC → IAQ →
%%          Baseline → PID → MPC → RL-DQN Controllers)
%%
%% REQUIREMENTS:
%%   - MATLAB R2021a or newer
%%   - Simulink (base)
%%   - All Phase 1-2 .mat files must exist in current folder
%%     OR run with GENERATE_DEMO_DATA = true to auto-create them
%%
%% OUTPUT:
%%   - Pune_HVAC_Complete.slx  (main model)
%%   - Fig_*  validation plots
%%   - All_Results.mat
%%
%% USAGE:
%%   >> Pune_HVAC_Full_Simulink        % requires real .mat files
%%   >> Pune_HVAC_Full_Simulink        % set GENERATE_DEMO_DATA=true below
%%                                       to run standalone with synthetic data
%% ================================================================

clearvars; clc; close all;

fprintf('================================================================\n');
fprintf('  PUNE 3BHK HVAC — FULL SIMULINK MODEL GENERATOR\n');
fprintf('  Phase 1-3  |  Days 1-10  |  4 Controllers\n');
fprintf('================================================================\n\n');

%% ────────────────────────────────────────────────────────────────
%%  USER SETTINGS
%% ────────────────────────────────────────────────────────────────
GENERATE_DEMO_DATA = true;   % true = create synthetic TMY+Building data
                              % false = load real .mat files from disk
RUN_SIMULATION     = true;   % true = sim() after building model
SIM_DAYS           = 365;    % days to simulate (7=quick test, 365=full year)
BUILD_SIMULINK     = true;   % true = build .slx model
PLOT_RESULTS       = true;   % true = generate figures

T_SET              = 24;     % °C cooling setpoint
T_NIGHT_OFFSET     = 2;      % °C night setback (setpoint + offset)
GRID_FACTOR        = 0.82;   % kgCO2/kWh Maharashtra CEA 2023

fprintf('Settings:\n');
fprintf('  Demo data:  %s\n', mat2str(GENERATE_DEMO_DATA));
fprintf('  Sim days:   %d\n', SIM_DAYS);
fprintf('  Build .slx: %s\n', mat2str(BUILD_SIMULINK));
fprintf('  Run sim:    %s\n\n', mat2str(RUN_SIMULATION));

%% ────────────────────────────────────────────────────────────────
%%  STEP 1: DATA PREPARATION
%% ────────────────────────────────────────────────────────────────
fprintf('──────────────────────────────────────────\n');
fprintf('STEP 1: Preparing simulation data\n');
fprintf('──────────────────────────────────────────\n');

if GENERATE_DEMO_DATA
    [PuneTMY, Building, Thermal, HVAC_Equip, IAQ_params, PM25_outdoor] = ...
        generate_demo_data();
    fprintf('  Demo data generated (synthetic Pune climate)\n');
else
    % Load real processed .mat files from Day1-Day6 scripts
    missing = {};
    files = {'Pune_TMY_processed.mat','Building_Pune_Params.mat',...
             'Thermal_Model_Params.mat','HVAC_Equipment_Params.mat',...
             'IAQ_Model_Params.mat'};
    for f = 1:length(files)
        if ~isfile(files{f}); missing{end+1} = files{f}; end
    end
    if ~isempty(missing)
        fprintf('  MISSING files:\n');
        for f = 1:length(missing); fprintf('    %s\n', missing{f}); end
        error('Run Day1-Day6 scripts first, or set GENERATE_DEMO_DATA=true');
    end
    load('Pune_TMY_processed.mat');
    load('Building_Pune_Params.mat');
    load('Thermal_Model_Params.mat');
    load('HVAC_Equipment_Params.mat');
    load('IAQ_Model_Params.mat');
    fprintf('  Real .mat files loaded\n');
end

N_SIM = SIM_DAYS * 24;   % simulation hours
N_SIM = min(N_SIM, length(PuneTMY.T_db));

%% ────────────────────────────────────────────────────────────────
%%  STEP 2: BUILD TIME-SERIES WORKSPACE VARIABLES
%% ────────────────────────────────────────────────────────────────
fprintf('\nSTEP 2: Building Simulink workspace time-series\n');

t_vec = (0:N_SIM-1)' * 3600;   % seconds

% Slice to simulation length
T_db    = PuneTMY.T_db(1:N_SIM);
RH_out  = PuneTMY.RH(1:N_SIM);
GHI_out = PuneTMY.GHI(1:N_SIM);
T_wb    = PuneTMY.T_wb(1:N_SIM);
season  = double(PuneTMY.season(1:N_SIM));

% Sol-air temperature (roof)
alpha_roof = 0.70; eps_roof = 0.90; h_out = 22.7; delta_R = 63;
T_sol_air = T_db + (alpha_roof.*GHI_out - eps_roof*delta_R) ./ h_out;

% Zone solar + internal gains
Q_sol_z1 = Thermal.Q_solar(1:N_SIM, 1);
Q_sol_z2 = Thermal.Q_solar(1:N_SIM, 2);
Q_sol_z3 = Thermal.Q_solar(1:N_SIM, 3);
Q_int_z1 = Thermal.Q_int(1:N_SIM, 1);
Q_int_z2 = Thermal.Q_int(1:N_SIM, 2);
Q_int_z3 = Thermal.Q_int(1:N_SIM, 3);

% PM2.5 outdoor profile
PM25_out  = PM25_outdoor(1:N_SIM);

% ToD tariff (24-hour pattern repeated)
tod_pattern = [5.20 5.20 5.20 5.20 5.20 5.20 ...  % 12-6AM off-peak
               8.95 8.95 8.95 ...                  % 7-9AM peak
               7.15 7.15 7.15 7.15 7.15 7.15 7.15 7.15 7.15 ...  % normal
               8.95 8.95 8.95 8.95 8.95 ...        % 7-11PM peak
               5.20];                               % midnight off-peak
tod_full  = repmat(tod_pattern', ceil(N_SIM/24), 1);
tod_full  = tod_full(1:N_SIM);

% IEC capacity profile
iec_cap = zeros(N_SIM, 1);
rho_air = 1.18; cp_air = 1006;
V_dot_iec = 0.35;   % m3/s
m_dot_iec = rho_air * V_dot_iec;
for i = 1:N_SIM
    if RH_out(i) < 50 && T_db(i) > 28
        T_sup = T_wb(i) + 2.0;
        dT    = max(0, T_db(i) - T_sup);
        iec_cap(i) = m_dot_iec * cp_air * dT;   % W
    end
end

% DOAS power profile (precomputed)
doas_P = zeros(N_SIM, 1);
for i = 1:N_SIM
    if RH_out(i) > 65 && T_db(i) > 22
        doas_P(i) = HVAC_Equip.doas.P_elec_kW(min(i,length(HVAC_Equip.doas.P_elec_kW)));
    end
end

% Pack into [t, data] matrices for From Workspace blocks
TS.T_outdoor   = [t_vec, T_db];
TS.T_sol_air   = [t_vec, T_sol_air];
TS.RH_outdoor  = [t_vec, RH_out];
TS.GHI         = [t_vec, GHI_out];
TS.T_wb        = [t_vec, T_wb];
TS.season      = [t_vec, season];
TS.tod_tariff  = [t_vec, tod_full];
TS.Q_sol_z1    = [t_vec, Q_sol_z1];
TS.Q_sol_z2    = [t_vec, Q_sol_z2];
TS.Q_sol_z3    = [t_vec, Q_sol_z3];
TS.Q_int_z1    = [t_vec, Q_int_z1];
TS.Q_int_z2    = [t_vec, Q_int_z2];
TS.Q_int_z3    = [t_vec, Q_int_z3];
TS.iec_cap     = [t_vec, iec_cap];
TS.doas_P      = [t_vec, doas_P];
TS.PM25_out    = [t_vec, PM25_out];

save('SimWS_Full.mat', 'TS', 'Thermal', 'HVAC_Equip');
fprintf('  Saved: SimWS_Full.mat  (%d hours)\n', N_SIM);

%% ────────────────────────────────────────────────────────────────
%%  STEP 3: MODEL PARAMETERS
%% ────────────────────────────────────────────────────────────────
fprintf('\nSTEP 3: Computing model parameters\n');

% Zone UA and C values
for z = 1:3
    P.C(z)       = Thermal.zone(z).C_total;
    P.UA_wall(z) = Thermal.zone(z).UA_wall;
    P.UA_roof(z) = Thermal.zone(z).UA_roof;
    P.UA_glz(z)  = Thermal.zone(z).UA_glz;
    P.UA_inf(z)  = Thermal.zone(z).UA_inf;
    P.UA_tot(z)  = Thermal.zone(z).UA_total;
    P.tau_h(z)   = Thermal.zone(z).tau_hours;
end

P.R_z1z2 = Thermal.R_z1z2;
P.R_z2z3 = Thermal.R_z2z3;
P.T_init  = 26;       % °C

% VRF parameters
P.vrf_Qmax   = HVAC_Equip.vrf.Q_rated_kW * 1000;   % W
P.vrf_Qmin   = P.vrf_Qmax * 0.20;                   % W
P.vrf_COP    = HVAC_Equip.vrf.COP_rated;
P.vrf_T_ref  = HVAC_Equip.vrf.T_ref_rated;
P.vrf_cop_a  = HVAC_Equip.vrf.cop_a;
P.vrf_cop_b  = HVAC_Equip.vrf.cop_b;
P.vrf_T_min  = HVAC_Equip.vrf.T_out_min;
P.vrf_T_max  = HVAC_Equip.vrf.T_out_max;

% IEC
P.iec_eff    = HVAC_Equip.iec.eff_wb;
P.iec_Pfan   = HVAC_Equip.iec.P_fan_kW * 1000;      % W
P.iec_RH_max = HVAC_Equip.iec.RH_enable;
P.iec_T_min  = HVAC_Equip.iec.T_enable;

% % PID parameters (Zone 2 primary — distributed to all zones)
% P.pid_Kp   = [150,  250,  100];    % W/°C
% P.pid_Ti   = [7200, 7200, 7200];   % s
% P.pid_Td   = [900,  900,  900];    % s
% P.pid_Kd   = P.pid_Kp .* P.pid_Td;
% P.pid_Ki   = P.pid_Kp ./ P.pid_Ti;
% P.pid_iLim = [5000, 8000, 4000];   % W anti-windup limit
% P.pid_db   = 0.5;                  % °C deadband
% PID parameters — replace existing block
% P.pid_Kp   = [180,  300,  120];    % W/°C  — moderate, not aggressive
% P.pid_Ti   = [2400, 2400, 2400];   % s  — 40 min, balanced for tau=2.5h
% P.pid_Td   = [120,  120,  120];    % s  — small derivative to avoid kick
% P.pid_Kd   = P.pid_Kp .* P.pid_Td;
% P.pid_Ki   = P.pid_Kp ./ P.pid_Ti;
% P.pid_iLim = [3000, 5000, 2000];   % W  — MUST be < zone share of Qmax
% P.pid_db   = 0.4;                  % °C deadband
% Step 3 — replace PID block
% P.pid_Kp   = [400,  700,  300];    % W/°C — balanced for tau=63h
% P.pid_Ti   = [5400, 5400, 5400];   % s — 1.5h integral
% P.pid_Td   = [300,  300,  300];    % s
% P.pid_Kd   = P.pid_Kp .* P.pid_Td;
% P.pid_Ki   = P.pid_Kp ./ P.pid_Ti;
% P.pid_iLim = [3500, 4800, 2200];   % W — below zone Qmax share
% P.pid_db   = 0.6;                  % °C — wider deadband, less hunting

% Step 3 — PID parameters
P.pid_Kp   = [600,  1000,  450];   % W/°C
P.pid_Ti   = [5400, 5400,  5400];  % s
P.pid_Td   = [300,  300,   300];   % s
P.pid_Kd   = P.pid_Kp .* P.pid_Td;
P.pid_Ki   = P.pid_Kp ./ P.pid_Ti;
P.pid_iLim = [4000, 5000,  2500];  % W
P.pid_db   = 0.5;                  % °C
% % MPC parameters
% P.mpc_Hp   = 12;    % prediction horizon (hours)
% P.mpc_Hc   = 3;     % control horizon
% P.mpc_w_c  = 300;   % comfort weight
% P.mpc_w_e  = 0.2;   % energy weight
% P.mpc_w_r  = 1.5;   % rate weight
% P.mpc_a    = 0.95;  % discrete-time pole (tau~20h)
% P.mpc_b    = 3.0*(1-0.95)/P.UA_tot(2);   % control gain
% MPC parameters  — replace existing block
P.mpc_Hp   = 24;    % longer horizon sees full diurnal cycle
P.mpc_Hc   = 4;
P.mpc_w_c  = 500;
P.mpc_w_e  = 0.15;
P.mpc_w_r  = 1.0;
P.mpc_a    = 0.67;  % matches actual zone pole exp(-1/2.5)
P.mpc_b    = 3.0*(1-0.67)/P.UA_tot(2);

% RL state normalization bounds
P.rl_T_sp   = T_SET;
P.rl_gamma  = 0.97;
P.rl_n_act  = 6;
P.rl_actions = P.vrf_Qmax * [0, 0.15, 0.30, 0.50, 0.75, 1.00];

%% Hybrid Controller Parameters
P.hybrid_alpha_init = [0.60, 0.35, 0.05];  % [MPC, PID, safety] initial weights
P.hybrid_lr         = 0.01;   % weight adaptation learning rate
P.hybrid_mem        = 48;     % hours of performance memory
P.hybrid_T_tol      = 1.0;    % °C — tolerance for weight update
P.hybrid_n_states   = 6;      % state features for weight selector
P.hybrid_Qmax       = P.vrf_Qmax;
P.hybrid_db         = 0.3;    % °C deadband

fprintf('  Hybrid: MPC+PID+RL fusion  α_init=[%.2f,%.2f,%.2f]\n',...
    P.hybrid_alpha_init(1), P.hybrid_alpha_init(2), P.hybrid_alpha_init(3));

% IAQ
V_zone = sum(arrayfun(@(z) Thermal.zone(z).volume, 1:3));
P.V_total   = V_zone;
P.Q_inf_m3s = HVAC_Equip.Q_inf_m3s;
P.Q_oa_m3s  = HVAC_Equip.doas.V_oa_m3s;
P.CO2_out   = 420;
P.CO2_gen   = 4 * 0.0052e-3;   % m3/s (4 people)
P.eta_HEPA  = 0.9995;
P.lambda_dep= 0.5/3600;
P.G_PM_cook = 150/3600;
P.G_PM_bg   = 5/3600;
P.Q_recirc  = 0.30;
P.zone_frac = Thermal.zone_frac;

fprintf('  Zone time constants: Z1=%.1fh  Z2=%.1fh  Z3=%.1fh\n', ...
    P.tau_h(1), P.tau_h(2), P.tau_h(3));
fprintf('  VRF: %.1f kW rated  COP=%.1f @ %d°C\n', ...
    P.vrf_Qmax/1000, P.vrf_COP, P.vrf_T_ref);
fprintf('  PID: Kp=%g/%g/%g W/°C  Ti=%gs\n', ...
    P.pid_Kp(1), P.pid_Kp(2), P.pid_Kp(3), P.pid_Ti(1));
fprintf('  MPC: Hp=%d  Hc=%d  a=%.3f\n', P.mpc_Hp, P.mpc_Hc, P.mpc_a);


% Diagnostic: steady-state zone temp at peak outdoor
for z = 1:3
    T_ss = 41 + (Q_int_z2(14*24) + Q_sol_z2(14*24)) / P.UA_tot(z);
    fprintf('  Zone %d SS temp at 41C outdoor: %.1f C\n', z, T_ss);
end
% Cooling capacity needed to hold 24C at peak
Q_needed = P.UA_tot(2) * (41 - 24) + Q_sol_z2(14*24) + Q_int_z2(14*24);
fprintf('  Peak cooling needed Zone 2: %.0f W vs VRF max: %.0f W\n',...
    Q_needed, P.vrf_Qmax * P.zone_frac(2));

%% ────────────────────────────────────────────────────────────────
%%  STEP 4: PRE-TRAIN RL NETWORK (pure MATLAB, no RL Toolbox)
%% ────────────────────────────────────────────────────────────────
fprintf('\nSTEP 4: Pre-training RL-DQN network\n');

rng(42);
n_s   = 8;    % state features
n_a   = P.rl_n_act;
arch  = [n_s, 128, 128, n_a];
n_lay = length(arch)-1;

W = cell(n_lay,1); b_nn = cell(n_lay,1);
for l = 1:n_lay
    sc = sqrt(2/arch(l));
    W{l} = randn(arch(l+1), arch(l)) * sc;
    b_nn{l} = zeros(arch(l+1), 1);
end
Wt = W; bt = b_nn;   % target network

% Training parameters
gamma   = P.rl_gamma;
% lr      = 3e-4;
eps     = 1.0;
eps_min = 0.05;
% eps_dec = 0.993;
buf_sz  = 15000;
batch   = 128;
% n_ep    = 500;   % episodes (increase for better performance)
% ep_len  = 168;   % 1 week per episode
n_ep    = 800;   % more episodes for better convergence
ep_len  = 336;   % 2 weeks per episode (sees more diversity)
lr      = 2e-4;  % slightly lower lr for stability
eps_dec = 0.996; % slower decay — more exploration early

% Replay buffer
buf_s  = zeros(buf_sz, n_s);
buf_a  = zeros(buf_sz, 1);
buf_r  = zeros(buf_sz, 1);
buf_ns = zeros(buf_sz, n_s);
buf_d  = zeros(buf_sz, 1);
buf_ptr= 1; buf_cnt = 0;

ep_R = zeros(n_ep,1); ep_C = zeros(n_ep,1);
best_R = -inf; nn_best_W = W; nn_best_b = b_nn;

fprintf('  Training %d episodes × %d hours ...\n', n_ep, ep_len);
season_starts = [48, 2900, 4400, 6600];
tic_rl = tic;

for ep = 1:n_ep
    s_idx  = season_starts(mod(ep-1,4)+1) + randi(200);
    s_idx  = min(s_idx, N_SIM - ep_len - 2);
    Tz_ep  = [26 26 26]';
    sum_R  = 0; sum_C = 0; u_prev_rl = 0;

    for t = 1:ep_len
        hi  = s_idx + t; if hi > N_SIM; hi = mod(hi,N_SIM)+1; end
        T_o = T_db(hi); RH_o = RH_out(hi);
        h24 = mod(hi-1,24)+1;
        trf = tod_full(hi);
        sea = season(hi);
        sv  = rl_state(Tz_ep(2), T_o, RH_o, h24, sea, trf, T_SET);

        % Epsilon-greedy action
        if rand() < eps
            ai = randi(n_a);
        else
            Qv = nn_forward(W, b_nn, sv);
            [~,ai] = max(Qv);
        end
        % Curriculum: force cooling if very hot
        % if Tz_ep(2) > 28 && ai == 1; ai = randi([2,n_a]); end
        % if Tz_ep(2) < 21 && ai > 1;  ai = 1; end
        % Curriculum: force action when clearly outside comfort
    if Tz_ep(2) > 27 && ai == 1
        ai = randi([3, n_a]);   % force moderate-to-full cooling
    end
    if Tz_ep(2) < 22 && ai > 2
        ai = randi([1, 2]);     % force off or minimal
    end

        u_c = P.rl_actions(ai);
        du  = u_c - u_prev_rl;
        if abs(du) > P.vrf_Qmax*0.35
            u_c = u_prev_rl + sign(du)*P.vrf_Qmax*0.35;
        end
        u_c = max(0, min(P.vrf_Qmax, u_c));
        u_prev_rl = u_c;

        r   = rl_reward(Tz_ep(2), u_c, trf, T_SET, P.vrf_Qmax);
        sum_R = sum_R + r;
        if Tz_ep(2) >= 22 && Tz_ep(2) <= 26; sum_C = sum_C+1; end

        % Zone update
        Tz_ep = zone_euler(Tz_ep, T_o, T_sol_air(hi), hi, ...
                           -u_c*P.zone_frac', P, Q_sol_z1, Q_sol_z2, Q_sol_z3, ...
                           Q_int_z1, Q_int_z2, Q_int_z3, 3600);

        % Next state
        hi2 = min(s_idx+t+1, N_SIM);
        sv2 = rl_state(Tz_ep(2), T_db(hi2), RH_out(hi2), mod(hi2-1,24)+1,...
                       season(hi2), tod_full(hi2), T_SET);
        done = (t == ep_len);

        % Store
        buf_s(buf_ptr,:)  = sv';
        buf_a(buf_ptr)    = ai;
        buf_r(buf_ptr)    = r;
        buf_ns(buf_ptr,:) = sv2';
        buf_d(buf_ptr)    = double(done);
        buf_ptr = mod(buf_ptr, buf_sz)+1;
        buf_cnt = min(buf_cnt+1, buf_sz);

        % Train batch
        if buf_cnt >= batch
            idx = randperm(buf_cnt, batch);
            targets = buf_r(idx);
            for bi = 1:batch
                if ~buf_d(idx(bi))
                    Qn = nn_forward(Wt, bt, buf_ns(idx(bi),:)');
                    targets(bi) = targets(bi) + gamma*max(Qn);
                end
            end
            for bi = 1:batch
                [W, b_nn] = nn_sgd_step(W, b_nn, buf_s(idx(bi),:)', ...
                                         buf_a(idx(bi)), targets(bi), lr, batch);
            end
        end
    end

    if mod(ep,10)==0; Wt=W; bt=b_nn; end
    eps = max(eps_min, eps*eps_dec);
    ep_R(ep) = sum_R/ep_len;
    ep_C(ep) = sum_C/ep_len*100;
    if ep_R(ep) > best_R; best_R=ep_R(ep); nn_best_W=W; nn_best_b=b_nn; end
    if mod(ep,20)==0
        fprintf('    Ep%3d/%d | eps=%.3f | R=%6.3f | Comfort=%.0f%%\n',...
            ep,n_ep,eps,mean(ep_R(max(1,ep-19):ep)),mean(ep_C(max(1,ep-19):ep)));
    end
end
t_rl = toc(tic_rl);
fprintf('  RL training complete: %.1f min  Best R=%.3f\n', t_rl/60, best_R);

% Save RL weights to MAT file (used by Simulink MATLAB Function block)
RL_W = nn_best_W; RL_b = nn_best_b;
save('RL_Network_Weights.mat', 'RL_W', 'RL_b', 'P');
fprintf('  Saved: RL_Network_Weights.mat\n');

%% ────────────────────────────────────────────────────────────────
%%  STEP 5: PURE-MATLAB ANNUAL SIMULATION (Validation)
%%  (Runs always, gives results even if Simulink build is skipped)
%% ────────────────────────────────────────────────────────────────
fprintf('\nSTEP 5: Pure-MATLAB annual simulation (validation)\n');

% ctrl_names = {'Baseline','PID','MPC','RL-DQN'};
% ctrl_ids   = {'baseline','pid','mpc','rl'};
ctrl_names = {'Baseline','PID','MPC','RL-DQN','Hybrid'};
ctrl_ids   = {'baseline','pid','mpc','rl','hybrid'};
RESULTS    = struct();

for ci = 1:5
    cid = ctrl_ids{ci};
    fprintf('  Simulating %s ...', ctrl_names{ci});
    tic_c = tic;

    Tz   = [28 28 28]';     % realistic January equilibrium
    % Tz   = [26 26 26]';
    co2  = 450; pm = 15;
    integ= [0 0 0]; e_prev=[0 0 0];
    u_prev_c = 0;


     %% Hybrid controller state — initialized fresh for each controller run
    % hyb_integ     = zeros(3,1);
    % hyb_eprev     = zeros(3,1);
    % hyb_umpc_prev = 0;
    % hyb_alpha     = P.hybrid_alpha_init(:);   % column vector [0.60;0.35;0.05]
    % hyb_mpc_err_hist = zeros(P.hybrid_mem, 1);
    % hyb_pid_err_hist = zeros(P.hybrid_mem, 1);
    % hyb_hist_ptr  = 1;
    % hyb_step      = 0;
    
    %% Hybrid controller state
    hyb_integ     = zeros(3,1);
    hyb_eprev     = zeros(3,1);
    hyb_umpc_prev = 0;
    hyb_alpha     = P.hybrid_alpha_init(:);
    hyb_mpc_err_hist = zeros(P.hybrid_mem, 1);
    hyb_pid_err_hist = zeros(P.hybrid_mem, 1);
    hyb_hist_ptr  = 1;
    hyb_step      = 0;
    hyb_T_prev    = 28.0;   % initial zone temperature matches Tz init
    T_lk          = 0;      % lookahead temperature (initialized safely)
    
%     %% ── Burn-in: run 500h to reach thermal equilibrium before logging
% BURN_IN = 500;
% for bi = 1:BURN_IN
%     T_o_bi  = T_db(mod(bi-1,N_SIM)+1);
%     T_sa_bi = T_sol_air(mod(bi-1,N_SIM)+1);
%     h24_bi  = mod(bi-1,24)+1;
%     SP_bi   = T_SET;
%     T_avg_bi= mean(Tz);
%     dT_bi   = max(Tz) - SP_bi;
% 
%     % All controllers: run a moderate cooling during burn-in
%     Q_bi = 0;
%     if dT_bi > 1.0;     Q_bi = P.vrf_Qmax * 0.50;
%     elseif dT_bi > 0.0; Q_bi = P.vrf_Qmax * 0.20;
%     end
%     Q_hvac_bi = -Q_bi * P.zone_frac';
%     Tz = zone_euler(Tz, T_o_bi, T_sa_bi, mod(bi-1,N_SIM)+1, Q_hvac_bi, P,...
%                     Q_sol_z1, Q_sol_z2, Q_sol_z3,...
%                     Q_int_z1, Q_int_z2, Q_int_z3, 3600);
%     end
%     fprintf('  Burn-in complete: Tz=[%.1f, %.1f, %.1f]°C\n', Tz(1),Tz(2),Tz(3));

    Tz1_log = zeros(N_SIM,1); Tz2_log = zeros(N_SIM,1);
    Tz3_log = zeros(N_SIM,1); CO2_log = zeros(N_SIM,1);
    PM_log  = zeros(N_SIM,1); Pvrf_log= zeros(N_SIM,1);
    Piec_log= zeros(N_SIM,1); Pdoas_log=zeros(N_SIM,1);
    mode_log= zeros(N_SIM,1);
    Q_cool_log = zeros(N_SIM,1);

    for i = 1:N_SIM
        T_o  = T_db(i);
        T_sa = T_sol_air(i);
        RH_o = RH_out(i);
        h24  = mod(i-1,24)+1;
        trf  = tod_full(i);
        sea  = season(i);
        T_avg= mean(Tz);

        % Night setpoint
        is_night = (h24 >= 23 || h24 <= 6);
        SP = T_SET + T_NIGHT_OFFSET*double(is_night);

        Q_cool = 0;   % W (positive = cooling extracted)

        switch cid
            % case 'baseline'
            %     if T_o > 35
            %         Q_cool = P.vrf_Qmax * min(1.1, max(0.2,(T_avg-SP)/3));
            %     elseif T_o > 28 && RH_o < 50
            %         Q_cool = P.vrf_Qmax * 0.5;
            %     elseif RH_o > 65 && T_o > 22
            %         Q_cool = P.vrf_Qmax * min(1.0, max(0.2,(T_avg-SP)/3));
            %     elseif T_avg > SP + 0.5
            %         Q_cool = P.vrf_Qmax * min(0.7, max(0.2,(T_avg-SP)/5));
            %     end
            %     mode_log(i) = mode_from_T_RH(T_o, RH_o, T_avg, SP);
               
    %         case 'baseline'
    % % Simple ON/OFF thermostat — no intelligence whatsoever
    % % Represents typical Indian home AC remote control behavior
    % if T_avg > SP + 1.0
    %     % Zone clearly hot — run at fixed 70%
    %     Q_cool = P.vrf_Qmax * 0.70;
    % elseif T_avg > SP + 0.5
    %     % Mild overshoot — run at 45%
    %     Q_cool = P.vrf_Qmax * 0.45;
    % elseif T_avg < SP - 1.0
    %     % Zone already cool enough — OFF
    %     Q_cool = 0;
    % else
    %     % Dead zone — maintain last state (simplified: off)
    %     Q_cool = 0;
    % end
    %     % NO tariff awareness
    % % NO pre-cooling
    % % NO weather forecast
    % % NO zone-specific control
    % mode_log(i) = mode_from_T_RH(T_o, RH_o, T_avg, SP);
            
    %         case 'baseline'
    % % Stateless 2-point thermostat — NO persistent variables
    % % Represents typical Indian split AC with remote thermostat
    % dT = T_avg - SP;
    % if dT > 2.0
    %     Q_cool = P.vrf_Qmax * 0.90;   % very hot — near full blast
    % elseif dT > 1.0
    %     Q_cool = P.vrf_Qmax * 0.70;   % hot — 70%
    % elseif dT > 0.5
    %     Q_cool = P.vrf_Qmax * 0.45;   % warm — 45%
    % elseif dT > 0.0
    %     Q_cool = P.vrf_Qmax * 0.20;   % just above SP — minimum
    % else
    %     Q_cool = 0;                    % at or below SP — off
    % end
    % mode_log(i) = mode_from_T_RH(T_o, RH_o, T_avg, SP);
            

    %     case 'baseline'
    % SP = T_SET;   % baseline uses FIXED setpoint — no night setback
    % T_err_bl = T_avg - SP;
    % if     T_err_bl > 3.0;  Q_cool = P.vrf_Qmax * 1.00;
    % elseif T_err_bl > 2.0;  Q_cool = P.vrf_Qmax * 0.85;
    % elseif T_err_bl > 1.0;  Q_cool = P.vrf_Qmax * 0.65;
    % elseif T_err_bl > 0.5;  Q_cool = P.vrf_Qmax * 0.40;
    % elseif T_err_bl > 0.0;  Q_cool = P.vrf_Qmax * 0.20;
    % else;                   Q_cool = 0;
    % end
    % mode_log(i) = mode_from_T_RH(T_o, RH_o, T_avg, SP);

    %     case 'baseline'
    % % Indian room AC: fixed setpoint 24°C, simple on/off with deadband
    % % No persistent needed — use Q_cool carry-forward via u_prev_c
    % SP_bl = T_SET;   % fixed 24°C always, no night setback
    % dT_bl = T_avg - SP_bl;
    % 
    % % Proportional bang-bang with 5 levels
    % if dT_bl >= 4.0
    %     Q_cool = P.vrf_Qmax * 1.00;
    % elseif dT_bl >= 2.5
    %     Q_cool = P.vrf_Qmax * 0.80;
    % elseif dT_bl >= 1.5
    %     Q_cool = P.vrf_Qmax * 0.60;
    % elseif dT_bl >= 0.8
    %     Q_cool = P.vrf_Qmax * 0.40;
    % elseif dT_bl >= 0.2
    %     Q_cool = P.vrf_Qmax * 0.20;
    % else
    %     Q_cool = 0;
    % end

    % Key difference from smart controllers:
    % No tariff awareness — runs same at 5.20 and 8.95 Rs/kWh
    % No pre-cooling — purely reactive
    % No zone-specific control — uses zone average only
    % mode_log(i) = mode_from_T_RH(T_o, RH_o, T_avg, SP_bl);
    % 
    % case 'baseline'
    % SP_bl = T_SET;  % fixed 24°C, no night setback
    % 
    % % Use MAX zone temp not average — homeowner feels the hottest room
    % T_max_zone = max(Tz);
    % dT_bl = T_max_zone - SP_bl;
    % 
    % % Simple stepped response
    % if     dT_bl >= 4.0;  Q_cool = P.vrf_Qmax * 1.00;
    % elseif dT_bl >= 2.5;  Q_cool = P.vrf_Qmax * 0.80;
    % elseif dT_bl >= 1.5;  Q_cool = P.vrf_Qmax * 0.60;
    % elseif dT_bl >= 0.5;  Q_cool = P.vrf_Qmax * 0.35;
    % elseif dT_bl >= 0.0;  Q_cool = P.vrf_Qmax * 0.15;
    % else;                 Q_cool = 0;
    % end
    % 
    % mode_log(i) = mode_from_T_RH(T_o, RH_o, T_max_zone, SP_bl);
    
    % case 'baseline'
    %     %% Simple Indian room AC thermostat
    %     %% Fixed setpoint, no tariff awareness, no forecast, no modulation
    %     SP_bl  = T_SET;            % fixed 24°C always
    %     dT_bl  = T_avg - SP_bl;    % use mean zone temp — wall thermostat behavior
    % 
    %     if     dT_bl >= 3.5;  Q_cool = P.vrf_Qmax * 0.95;
    %     elseif dT_bl >= 2.0;  Q_cool = P.vrf_Qmax * 0.75;
    %     elseif dT_bl >= 1.0;  Q_cool = P.vrf_Qmax * 0.55;
    %     elseif dT_bl >= 0.5;  Q_cool = P.vrf_Qmax * 0.35;
    %     elseif dT_bl >= 0.1;  Q_cool = P.vrf_Qmax * 0.15;
    %     else;                 Q_cool = 0;
    %     end
    %     %% No tariff awareness — wastes energy at peak hours
    %     %% No pre-cooling — always reactive
    %     %% No zone-specific control — single thermostat
    %     mode_log(i) = mode_from_T_RH(T_o, RH_o, T_avg, SP_bl);

% case 'baseline'
%     %% Rule-based thermostat using OUTDOOR temperature as trigger
%     %% Real Indian homeowners turn AC on when it feels hot outside
%     %% Primary trigger: outdoor temp (always available, never drifts)
%     %% Secondary: zone average for fine control
%     SP_bl = T_SET;
% 
%     if T_o >= 35
%         %% Very hot outdoor — run hard regardless of zone temp
%         Q_cool = P.vrf_Qmax * 0.85;
%     elseif T_o >= 30
%         %% Hot outdoor — proportional to zone error
%         dT_bl = max(0, T_avg - SP_bl);
%         Q_cool = P.vrf_Qmax * min(0.75, max(0.30, dT_bl/5));
%     elseif T_o >= 26
%         %% Warm outdoor — light cooling if zone above SP
%         dT_bl = max(0, T_avg - SP_bl);
%         Q_cool = P.vrf_Qmax * min(0.50, max(0.15, dT_bl/6));
%     elseif T_avg > SP_bl + 0.8
%         %% Mild outdoor but zone still warm
%         Q_cool = P.vrf_Qmax * 0.20;
%     else
%         Q_cool = 0;
%     end
%     mode_log(i) = mode_from_T_RH(T_o, RH_o, T_avg, SP_bl);

    case 'baseline'
    SP_bl = T_SET;
    dT_bl = max(0, T_avg - SP_bl);

    % Primary: outdoor temperature trigger (like a homeowner)
    if T_o >= 35
        Q_cool = P.vrf_Qmax * 0.85;
    elseif T_o >= 30
        Q_cool = P.vrf_Qmax * min(0.75, max(0.30, dT_bl/4));
    elseif T_o >= 26
        Q_cool = P.vrf_Qmax * min(0.55, max(0.15, dT_bl/5));
    elseif dT_bl >= 2.0
        % Zone warm even in mild weather (internal gains)
        Q_cool = P.vrf_Qmax * 0.45;
    elseif dT_bl >= 0.8
        Q_cool = P.vrf_Qmax * 0.25;
    elseif dT_bl >= 0.2
        Q_cool = P.vrf_Qmax * 0.15;
    else
        Q_cool = 0;
    end
    mode_log(i) = mode_from_T_RH(T_o, RH_o, T_avg, SP_bl);

           % case 'pid'
            %     for z = 1:3
            %         err_z = max(0, Tz(z) - SP);
            %         if abs(err_z) < P.pid_db; err_z = 0; end
            %         integ(z) = clamp(integ(z)+err_z*3600, ...
            %                          -P.pid_iLim(z), P.pid_iLim(z));
            %         deriv_z  = (err_z - e_prev(z)) / 3600;
            %         e_prev(z)= err_z;
            %         Q_z = P.pid_Kp(z)*err_z + P.pid_Ki(z)*integ(z) + P.pid_Kd(z)*deriv_z;
            %         Q_z = max(0, min(P.vrf_Qmax*P.zone_frac(z)*1.1, Q_z));
            %         if Q_z < P.vrf_Qmin*P.zone_frac(z)*0.5
            %             Q_z = 0;
            %             integ(z) = integ(z) - err_z*3600;
            %         end
            %         Q_cool = Q_cool + Q_z;
            %     end
            %     mode_log(i) = 3;  % VRF primary
            
    %         case 'pid'
    % is_night_pid = (h24 >= 23 || h24 <= 6);
    % SP_pid = T_SET + T_NIGHT_OFFSET * double(is_night_pid);
    % for z = 1:3
    %     err_z = Tz(z) - SP_pid;
    % 
    %     % Deadband — no action near setpoint
    %     if abs(err_z) < P.pid_db
    %         err_z = 0;
    %     end
    % 
    %     % Cooling only — never command heating
    %     err_z = max(0, err_z);
    % 
    %     % Derivative on error
    %     deriv_z   = (err_z - e_prev(z)) / 3600;
    %     e_prev(z) = err_z;
    % 
    %     % Proportional + derivative first
    %     Q_z = P.pid_Kp(z)*err_z + P.pid_Kd(z)*deriv_z;
    % 
    %     % Add integral (with clamping BEFORE adding)
    %     Q_z = Q_z + P.pid_Ki(z) * integ(z);
    % 
    %     % Saturate output
    %     zone_max = P.vrf_Qmax * P.zone_frac(z);
    %     Q_z = max(0, min(zone_max, Q_z));
    % 
    %     % Anti-windup: only integrate when output is NOT saturated
    %     % This prevents integrator from winding up further at limits
    %     at_upper = (Q_z >= zone_max * 0.98);
    %     at_lower = (Q_z <= 0 && err_z <= 0);
    %     if ~at_upper && ~at_lower
    %         integ(z) = clamp(integ(z) + err_z * 3600, ...
    %                          0, P.pid_iLim(z));
    %     end
    % 
    %     % Min-on threshold: ignore tiny demands
    %     if Q_z < 200
    %         Q_z = 0;
    %     end
    % 
    %     Q_cool = Q_cool + Q_z;
    % end
    % Q_cool = min(Q_cool, P.vrf_Qmax);
    % mode_log(i) = 3;
            
    % case 'pid'
    % is_night_pid = (h24 >= 23 || h24 <= 6);
    % SP_pid = T_SET + T_NIGHT_OFFSET * double(is_night_pid);
    % for z = 1:3
    %     err_z = Tz(z) - SP_pid;
    %     if abs(err_z) < P.pid_db; err_z = 0; end
    %     err_z = max(0, err_z);  % cooling only
    % 
    %     deriv_z   = (err_z - e_prev(z)) / 3600;
    %     e_prev(z) = err_z;
    % 
    %     Q_z = P.pid_Kp(z)*err_z + P.pid_Ki(z)*integ(z) + P.pid_Kd(z)*deriv_z;
    %     zone_max = P.vrf_Qmax * P.zone_frac(z);
    %     Q_z = max(0, min(zone_max, Q_z));
    % 
    %     % Anti-windup
    %     if Q_z < zone_max*0.97 && err_z > 0
    %         integ(z) = clamp(integ(z) + err_z*3600, 0, P.pid_iLim(z));
    %     elseif err_z == 0
    %         integ(z) = integ(z) * 0.95;  % faster bleed at setpoint
    %     end
    % 
    %     % Tariff-aware: reduce output 20% at peak tariff
    %     if trf >= 8.95; Q_z = Q_z * 0.82; end
    % 
    %     if Q_z < 200; Q_z = 0; end
    %     Q_cool = Q_cool + Q_z;
    % end
    % Q_cool = min(Q_cool, P.vrf_Qmax * 0.95);  % cap below full rated
    % mode_log(i) = 3;
            
    case 'pid'
    is_night_pid = (h24 >= 23 || h24 <= 6);
    SP_pid = T_SET + T_NIGHT_OFFSET * double(is_night_pid);
    for z = 1:3
        err_z = Tz(z) - SP_pid;
        if abs(err_z) < P.pid_db; err_z = 0; end
        err_z = max(0, err_z);

        deriv_z   = (err_z - e_prev(z)) / 3600;
        e_prev(z) = err_z;

        Q_z = P.pid_Kp(z)*err_z + P.pid_Ki(z)*integ(z) + P.pid_Kd(z)*deriv_z;
        zone_max = P.vrf_Qmax * P.zone_frac(z);
        Q_z = max(0, min(zone_max, Q_z));

        % Clean anti-windup — no tariff cut here
        if Q_z < zone_max*0.97 && err_z > 0
            integ(z) = clamp(integ(z) + err_z*3600, 0, P.pid_iLim(z));
        elseif err_z == 0
            integ(z) = integ(z) * 0.97;
        end

        if Q_z < 150; Q_z = 0; end
        Q_cool = Q_cool + Q_z;
    end
    % Tariff awareness: modest reduction only at extreme peak
    % Does NOT cut comfort — only trims marginal excess
    if trf >= 8.95 && Q_cool > P.vrf_Qmax * 0.70
        Q_cool = Q_cool * 0.90;  % only 10% trim, only when already high
    end
    Q_cool = min(Q_cool, P.vrf_Qmax * 0.95);
    mode_log(i) = 3;     

            % case 'mpc'
            %     T_now = Tz(2);
            %     if T_now <= SP + 0.3
            %         Q_cool = 0;
            %     else
            %         % Simplified QP: gradient descent on J over Hp steps
            %         a = P.mpc_a; b_c = P.mpc_b;
            %         sum_e = 0; xk = T_now;
            %         for k = 1:P.mpc_Hp
            %             ik = min(i+k-1, N_SIM);
            %             xk = a*xk + 0.00015*(T_db(ik)-xk) ...
            %                + 0.00008*(T_sol_air(ik)-xk) ...
            %                + 1e-6*(Q_sol_z2(ik)+Q_int_z2(ik));
            %             w_k = 1 + 0.05*(P.mpc_Hp-k);  % decay weight
            %             sum_e = sum_e + (xk-SP)*w_k;
            %         end
            %         tod_w = 0.5 + 0.3*(trf/7.15);
            %         % Q_cool = max(0, min(P.vrf_Qmax*1.05, ...
            %         Q_cool = max(0, min(P.vrf_Qmax*1.2, ...));
            %                      150*sum_e/P.mpc_Hp/(1+tod_w*0.2)));
            %         if T_now > SP+2; Q_cool = max(Q_cool, P.vrf_Qmax*0.3); end
            %         du = Q_cool - u_prev_c;
            %         if abs(du) > P.vrf_Qmax*0.25
            %             Q_cool = u_prev_c + sign(du)*P.vrf_Qmax*0.25;
            %         end
            %         mode_log(i) = 3;
            %     end
            %     u_prev_c = Q_cool;
            
    %         case 'mpc'
    % is_night_mpc = (h24 >= 23 || h24 <= 6);
    % SP_mpc = T_SET + T_NIGHT_OFFSET * double(is_night_mpc);
    % 
    % % Pre-cooling logic: if next 3h outdoor temp rising, cool now
    % T_future_max = 0;
    % for kk = 1:6
    %     ik = min(i+kk, N_SIM);
    %     T_future_max = max(T_future_max, T_db(ik));
    % end
    % pre_cool_boost = max(0, (T_future_max - 32) / 10);  % 0 to ~0.9
    % 
    % T_now = Tz(2);
    % err_now = T_now - SP_mpc;
    % 
    % if T_now <= SP_mpc + 0.15
    %     Q_cool = 0;
    %     u_prev_c = 0;
    % else
    %     a_mpc = 0.67;
    %     sum_e = 0; xk = T_now;
    %     for k = 1:P.mpc_Hp
    %         ik = min(i+k-1, N_SIM);
    %         T_oa_k = T_db(ik);
    %         xk = a_mpc * xk + (1-a_mpc) * (T_oa_k + 3.0);
    %         w_k = power(0.92, k-1);
    %         sum_e = sum_e + max(0, xk - SP_mpc) * w_k;
    %     end
    %     % Reduce at peak tariff, increase at off-peak
    %     tod_w = 1.0 + 0.3 * ((trf - 7.15) / 3.75);
    %     Q_cool = (1400 * sum_e / P.mpc_Hp + ...
    %               pre_cool_boost * P.vrf_Qmax * 0.3) / tod_w;
    %     % Proportional floor
    %     Q_cool = max(Q_cool, err_now * 800);
    %     % Hard floors
    %     if T_now > SP_mpc + 1.5; Q_cool = max(Q_cool, P.vrf_Qmax*0.45); end
    %     if T_now > SP_mpc + 3.0; Q_cool = max(Q_cool, P.vrf_Qmax*0.80); end
    %     if T_now > SP_mpc + 5.0; Q_cool = P.vrf_Qmax * 1.05; end
    %     % Rate limit
    %     du = Q_cool - u_prev_c;
    %     if abs(du) > P.vrf_Qmax * 0.30
    %         Q_cool = u_prev_c + sign(du) * P.vrf_Qmax * 0.30;
    %     end
    %     Q_cool = max(P.vrf_Qmin, min(P.vrf_Qmax*1.05, Q_cool));
    %     mode_log(i) = 3;
    % end
    % u_prev_c = Q_cool;
            
        
    %  case 'mpc'
    % is_night_mpc = (h24 >= 23 || h24 <= 6);
    % SP_mpc = T_SET + T_NIGHT_OFFSET * double(is_night_mpc);
    % T_now = Tz(2);
    % 
    % if T_now <= SP_mpc + 0.10
    %     Q_cool = 0;
    %     u_prev_c = 0;
    % else
    %     % Prediction with solar + internal gain disturbance
    %     a_mpc = 0.67;
    %     sum_e = 0; xk = T_now;
    %     for k = 1:P.mpc_Hp
    %         ik = min(i+k-1, N_SIM);
    %         d_dist = (Q_sol_z2(ik) + Q_int_z2(ik)) / P.C(2) * 3600 * 0.4;
    %         xk = a_mpc*xk + (1-a_mpc)*(T_db(ik) + 2.0) + d_dist;
    %         w_k = power(0.93, k-1);
    %         sum_e = sum_e + max(0, xk - SP_mpc) * w_k;
    %     end
    % 
    %     % Pre-cooling: look ahead 4h for rising outdoor temp
    %     T_lookahead = 0;
    %     for kk = 1:4
    %         ik = min(i+kk, N_SIM);
    %         T_lookahead = max(T_lookahead, T_db(ik));
    %     end
    %     precool = max(0, (T_lookahead - 30)/15) * P.vrf_Qmax * 0.20;
    % 
    %     % Tariff-aware dispatch
    %     if trf >= 8.95
    %         tod_factor = 1.25;   % reduce at peak
    %     elseif trf <= 5.20
    %         tod_factor = 0.78;   % increase at off-peak (pre-cool cheaply)
    %     else
    %         tod_factor = 1.00;
    %     end
    % 
    %     err_now = max(0, T_now - SP_mpc);
    %     Q_cool = (1100 * sum_e / P.mpc_Hp + precool) / tod_factor;
    %     Q_cool = max(Q_cool, err_now * 700);
    % 
    %     % Tiered hard floors
    %     if T_now > SP_mpc + 0.8;  Q_cool = max(Q_cool, P.vrf_Qmax*0.18); end
    %     if T_now > SP_mpc + 2.0;  Q_cool = max(Q_cool, P.vrf_Qmax*0.50); end
    %     if T_now > SP_mpc + 3.5;  Q_cool = max(Q_cool, P.vrf_Qmax*0.80); end
    %     if T_now > SP_mpc + 5.0;  Q_cool = P.vrf_Qmax; end
    % 
    %     % Rate limit
    %     du = Q_cool - u_prev_c;
    %     if abs(du) > P.vrf_Qmax*0.28
    %         Q_cool = u_prev_c + sign(du)*P.vrf_Qmax*0.28;
    %     end
    %     Q_cool = max(P.vrf_Qmin, min(P.vrf_Qmax, Q_cool));
    %     mode_log(i) = 3;
    % end
    % u_prev_c = Q_cool;
            
      case 'mpc'
    is_night_mpc = (h24 >= 23 || h24 <= 6);
    SP_mpc = T_SET + T_NIGHT_OFFSET * double(is_night_mpc);
    T_now = Tz(2);

    if T_now <= SP_mpc + 0.10
        Q_cool = 0;
        u_prev_c = 0;
    else
        a_mpc = 0.67;
        sum_e = 0; xk = T_now;
        for k = 1:P.mpc_Hp
            ik = min(i+k-1, N_SIM);
            d_dist = (Q_sol_z2(ik) + Q_int_z2(ik)) / P.C(2) * 3600 * 0.35;
            xk = a_mpc*xk + (1-a_mpc)*(T_db(ik) + 1.5) + d_dist;
            w_k = power(0.93, k-1);
            sum_e = sum_e + max(0, xk - SP_mpc) * w_k;
        end

        % % Pre-cooling: ONLY activate when off-peak AND zone already warm
        % % This prevents wasting energy cooling unnecessarily
        % T_lookahead = 0;
        % for kk = 1:4
        %     ik = min(i+kk, N_SIM);
        %     T_lookahead = max(T_lookahead, T_db(ik));
        % end
        % precool_needed = (T_lookahead > 33) && (T_now > SP_mpc + 0.5) ...
        %                  && (trf <= 5.20);   % ONLY at off-peak tariff
        % precool = double(precool_needed) * P.vrf_Qmax * 0.15;
        % 
        % % Tariff dispatch — stronger differentiation
        % if trf >= 8.95
        %     tod_factor = 1.30;   % cut 23% at peak — save energy
        % elseif trf <= 5.20
        %     tod_factor = 0.82;   % boost 22% at off-peak — pre-cool
        % else
        %     tod_factor = 1.00;
        % end
            
        %         % Replace tod_factor block with this:
        % if trf >= 8.95
        %     tod_factor = 1.40;   % cut 29% at peak — strong energy saving
        % elseif trf <= 5.20
        %     tod_factor = 0.72;   % boost 39% at off-peak — aggressive pre-cool
        % else
        %     tod_factor = 1.00;
        % end
                
                % Inside case 'mpc' — replace tod_factor block only:
        if trf >= 8.95
            tod_factor = 1.35;   % reduce 26% at peak
        elseif trf <= 5.20
            tod_factor = 0.88;   % increase only 14% at off-peak (was 0.72 — too aggressive)
        else
            tod_factor = 1.00;
        end

        % Pre-cooling: ONLY at off-peak AND zone is already above SP
        % (restore the gate that was removed)
        T_lookahead = 0;
        for kk = 1:6
            ik = min(i+kk, N_SIM);
            T_lookahead = max(T_lookahead, T_db(ik));
        end
        precool_needed = (T_lookahead > 31) && (trf <= 5.20) ...
                         && (T_now > SP_mpc + 0.3);   % MUST be warm to pre-cool
        precool = double(precool_needed) * P.vrf_Qmax * 0.18;

        err_now = max(0, T_now - SP_mpc);
        Q_cool = (900 * sum_e / P.mpc_Hp + precool) / tod_factor;
        Q_cool = max(Q_cool, err_now * 650);

        % Tiered floors — proportional response
        if T_now > SP_mpc + 0.8;  Q_cool = max(Q_cool, P.vrf_Qmax*0.15); end
        if T_now > SP_mpc + 2.0;  Q_cool = max(Q_cool, P.vrf_Qmax*0.45); end
        if T_now > SP_mpc + 3.5;  Q_cool = max(Q_cool, P.vrf_Qmax*0.75); end
        if T_now > SP_mpc + 5.0;  Q_cool = P.vrf_Qmax; end

        du = Q_cool - u_prev_c;
        if abs(du) > P.vrf_Qmax*0.28
            Q_cool = u_prev_c + sign(du)*P.vrf_Qmax*0.28;
        end
        Q_cool = max(P.vrf_Qmin, min(P.vrf_Qmax, Q_cool));
        mode_log(i) = 3;
    end
    u_prev_c = Q_cool;  

            case 'rl'
                sv = rl_state(Tz(2), T_o, RH_o, h24, sea, trf, T_SET);
                Qv = nn_forward(nn_best_W, nn_best_b, sv);
                [~,ai] = max(Qv);
                u_c = P.rl_actions(ai);
                % Soft constraint
                if abs(Tz(2)-T_SET) < 1.0; u_c = 0; end
                du = u_c - u_prev_c;
                if abs(du) > P.vrf_Qmax*0.35
                    u_c = u_prev_c + sign(du)*P.vrf_Qmax*0.35;
                end
                Q_cool = max(0, min(P.vrf_Qmax, u_c));
                u_prev_c = Q_cool;
                mode_log(i) = 3;
                 
   case 'hybrid'
    %% ══════════════════════════════════════════════════
    %% HYBRID ADAPTIVE CONTROLLER v2
    %% MPC: strategic load-shifting and pre-cooling
    %% PID: fast tactical error correction
    %% Fusion: context-aware dynamic weighting
    %% ══════════════════════════════════════════════════

    is_night_hyb = (h24 >= 23 || h24 <= 6);
    SP_hyb = T_SET + T_NIGHT_OFFSET * double(is_night_hyb);
    T_now_hyb = Tz(2);
    err_hyb = max(0, T_now_hyb - SP_hyb);

    %% ── MPC Layer ──────────────────────────────────────
    Q_mpc_hyb = 0;
    if T_now_hyb > SP_hyb + 0.10
        a_h = 0.67;
        sum_e_h = 0; xk_h = T_now_hyb;
        for k = 1:P.mpc_Hp
            ik = min(i+k-1, N_SIM);
            d_h = (Q_sol_z2(ik)+Q_int_z2(ik))/P.C(2)*3600*0.35;
            xk_h = a_h*xk_h + (1-a_h)*(T_db(ik)+1.5) + d_h;
            sum_e_h = sum_e_h + max(0, xk_h-SP_hyb)*power(0.93,k-1);
        end

        if trf >= 8.95;     tod_h = 1.35;
        elseif trf <= 5.20; tod_h = 0.88;
        else;               tod_h = 1.00;
        end

        T_lk = 0;
        for kk = 1:6
            T_lk = max(T_lk, T_db(min(i+kk,N_SIM)));
        end
        pc_h = double((T_lk>31) && (trf<=5.20) && (T_now_hyb>SP_hyb+0.3));

        Q_mpc_hyb = (900*sum_e_h/P.mpc_Hp + pc_h*P.vrf_Qmax*0.18) / tod_h;
        Q_mpc_hyb = max(Q_mpc_hyb, err_hyb*650);
        if T_now_hyb>SP_hyb+0.8; Q_mpc_hyb=max(Q_mpc_hyb,P.vrf_Qmax*0.15); end
        if T_now_hyb>SP_hyb+2.0; Q_mpc_hyb=max(Q_mpc_hyb,P.vrf_Qmax*0.45); end
        if T_now_hyb>SP_hyb+3.5; Q_mpc_hyb=max(Q_mpc_hyb,P.vrf_Qmax*0.75); end
        if T_now_hyb>SP_hyb+5.0; Q_mpc_hyb=P.vrf_Qmax; end

        du_h = Q_mpc_hyb - hyb_umpc_prev;
        if abs(du_h) > P.vrf_Qmax*0.28
            Q_mpc_hyb = hyb_umpc_prev + sign(du_h)*P.vrf_Qmax*0.28;
        end
        Q_mpc_hyb = max(P.vrf_Qmin, min(P.vrf_Qmax, Q_mpc_hyb));
    end
    hyb_umpc_prev = Q_mpc_hyb;

    %% ── PID Layer ──────────────────────────────────────
    Q_pid_hyb = 0;
    for z = 1:3
        err_z_h = max(0, Tz(z) - SP_hyb);
        if err_z_h < P.hybrid_db; err_z_h = 0; end

        d_h2 = (err_z_h - hyb_eprev(z)) / 3600;
        hyb_eprev(z) = err_z_h;

        zm_h = P.vrf_Qmax * P.zone_frac(z);
        Q_z_h = P.pid_Kp(z)*err_z_h + P.pid_Ki(z)*hyb_integ(z) + P.pid_Kd(z)*d_h2;
        Q_z_h = max(0, min(zm_h, Q_z_h));

        if Q_z_h < zm_h*0.97 && err_z_h > 0
            hyb_integ(z) = clamp(hyb_integ(z)+err_z_h*3600, 0, P.pid_iLim(z));
        elseif err_z_h == 0
            hyb_integ(z) = hyb_integ(z)*0.97;
        end
        if Q_z_h < 150; Q_z_h = 0; end
        Q_pid_hyb = Q_pid_hyb + Q_z_h;
    end
    Q_pid_hyb = min(Q_pid_hyb, P.vrf_Qmax*0.95);

    %% ── Context-Aware Dynamic Weight Selector ──────────
    %% Weights determined by operational context, not just history
    %% This is faster and more reliable than pure performance tracking

    if err_hyb > 4.0
        %% EMERGENCY: zone very hot — trust PID's aggressive response
        w_mpc = 0.25; w_pid = 0.70; w_saf = 0.05;

    elseif err_hyb > 2.0
        %% HOT: both needed — PID leads, MPC supports
        w_mpc = 0.40; w_pid = 0.55; w_saf = 0.05;

    elseif trf <= 5.20 && T_lk > 31
        %% OFF-PEAK + HOT INCOMING: MPC pre-cooling is most valuable
        w_mpc = 0.75; w_pid = 0.20; w_saf = 0.05;

    elseif trf >= 8.95 && err_hyb < 1.0
        %% PEAK TARIFF + COMFORTABLE: MPC reduces load, PID prevents drift
        w_mpc = 0.65; w_pid = 0.30; w_saf = 0.05;

    elseif sea == 3 && RH_o > 75
        %% MONSOON: MPC prediction less accurate (humidity effects)
        %% Trust PID more for reliable comfort
        w_mpc = 0.35; w_pid = 0.60; w_saf = 0.05;

    elseif err_hyb < 0.5
        %% COMFORTABLE: let MPC handle pre-cooling quietly
        w_mpc = 0.70; w_pid = 0.25; w_saf = 0.05;

    else
        %% DEFAULT: balanced operation
        w_mpc = 0.55; w_pid = 0.40; w_saf = 0.05;
    end

    %% Blend with adaptive history weights (50% context, 50% learned)
    %% Record errors for adaptive component
    hyb_mpc_err_hist(hyb_hist_ptr) = abs(T_now_hyb - SP_hyb);
    hyb_pid_err_hist(hyb_hist_ptr) = abs(T_avg - SP_hyb);
    hyb_hist_ptr = mod(hyb_hist_ptr, P.hybrid_mem) + 1;
    hyb_step = hyb_step + 1;

    if hyb_step > P.hybrid_mem
        mpc_r = mean(hyb_mpc_err_hist) + 0.1;
        pid_r = mean(hyb_pid_err_hist) + 0.1;
        inv_m = 1/mpc_r; inv_p = 1/pid_r; inv_s = 0.1;
        tot = inv_m + inv_p + inv_s;
        alpha_learned = [inv_m/tot; inv_p/tot; inv_s/tot];

        %% Update stored alpha
        hyb_alpha = (1-P.hybrid_lr)*hyb_alpha + P.hybrid_lr*alpha_learned;
        hyb_alpha = hyb_alpha / sum(hyb_alpha);

        %% Final blend: 50% context-rule + 50% learned history
        w_mpc_final = 0.50*w_mpc + 0.50*hyb_alpha(1);
        w_pid_final = 0.50*w_pid + 0.50*hyb_alpha(2);
        w_saf_final = 1 - w_mpc_final - w_pid_final;
        w_saf_final = max(0.02, w_saf_final);
    else
        %% First 48h: pure context rules (no history yet)
        w_mpc_final = w_mpc;
        w_pid_final = w_pid;
        w_saf_final = w_saf;
    end

    %% ── Fusion Output ───────────────────────────────────
    Q_safety_hyb = min(err_hyb * 600, P.vrf_Qmax*0.45);

    Q_cool = w_mpc_final * Q_mpc_hyb ...
           + w_pid_final * Q_pid_hyb ...
           + w_saf_final * Q_safety_hyb;

    %% Boost: if zone drifting up fast (derivative > 0.3°C/h), add extra
    T_drift = (T_now_hyb - hyb_T_prev) / 1.0;   % °C/hour
    if T_drift > 0.3 && T_now_hyb > SP_hyb
        Q_cool = Q_cool * (1 + 0.15*min(1, T_drift));
    end
    hyb_T_prev = T_now_hyb;

    Q_cool = max(0, min(P.vrf_Qmax, Q_cool));
    mode_log(i) = 3;     


    %  case 'hybrid'
    % %% ─────────────────────────────────────────────────────
    % %% HYBRID ADAPTIVE CONTROLLER
    % %% Layer 1 (MPC): Strategic 24h planning
    % %% Layer 2 (PID): Tactical real-time correction
    % %% Layer 3 (Adaptive): Performance-based weight blending
    % %% State variables carried via hyb_* (initialized before loop)
    % %% ─────────────────────────────────────────────────────
    % 
    % %% Night setpoint
    % is_night_hyb = (h24 >= 23 || h24 <= 6);
    % SP_hyb = T_SET + T_NIGHT_OFFSET * double(is_night_hyb);
    % T_now_hyb = Tz(2);
    % 
    % %% ── Layer 1: MPC Strategic Output ──────────────────
    % Q_mpc_hyb = 0;
    % if T_now_hyb > SP_hyb + 0.10
    %     a_mpc_h = 0.67;
    %     sum_e_h = 0;
    %     xk_h    = T_now_hyb;
    %     for k = 1:P.mpc_Hp
    %         ik = min(i+k-1, N_SIM);
    %         d_dist_h = (Q_sol_z2(ik)+Q_int_z2(ik)) / P.C(2) * 3600 * 0.35;
    %         xk_h = a_mpc_h*xk_h + (1-a_mpc_h)*(T_db(ik)+1.5) + d_dist_h;
    %         w_k_h = power(0.93, k-1);
    %         sum_e_h = sum_e_h + max(0, xk_h - SP_hyb) * w_k_h;
    %     end
    % 
    %     if     trf >= 8.95;  tod_h = 1.35;
    %     elseif trf <= 5.20;  tod_h = 0.88;
    %     else;                tod_h = 1.00;
    %     end
    % 
    %     T_look_h = 0;
    %     for kk = 1:6
    %         ik2 = min(i+kk, N_SIM);
    %         T_look_h = max(T_look_h, T_db(ik2));
    %     end
    %     pc_cond = (T_look_h > 31) && (trf <= 5.20) && (T_now_hyb > SP_hyb + 0.3);
    %     precool_h = double(pc_cond) * P.vrf_Qmax * 0.18;
    % 
    %     err_now_h = max(0, T_now_hyb - SP_hyb);
    %     Q_mpc_hyb = (900*sum_e_h/P.mpc_Hp + precool_h) / tod_h;
    %     Q_mpc_hyb = max(Q_mpc_hyb, err_now_h * 650);
    % 
    %     if T_now_hyb > SP_hyb+0.8;  Q_mpc_hyb = max(Q_mpc_hyb, P.vrf_Qmax*0.15); end
    %     if T_now_hyb > SP_hyb+2.0;  Q_mpc_hyb = max(Q_mpc_hyb, P.vrf_Qmax*0.45); end
    %     if T_now_hyb > SP_hyb+3.5;  Q_mpc_hyb = max(Q_mpc_hyb, P.vrf_Qmax*0.75); end
    %     if T_now_hyb > SP_hyb+5.0;  Q_mpc_hyb = P.vrf_Qmax; end
    % 
    %     du_h = Q_mpc_hyb - hyb_umpc_prev;
    %     if abs(du_h) > P.vrf_Qmax * 0.28
    %         Q_mpc_hyb = hyb_umpc_prev + sign(du_h)*P.vrf_Qmax*0.28;
    %     end
    %     Q_mpc_hyb = max(P.vrf_Qmin, min(P.vrf_Qmax, Q_mpc_hyb));
    % end
    % hyb_umpc_prev = Q_mpc_hyb;
    % 
    % %% ── Layer 2: PID Tactical Correction ──────────────
    % Q_pid_hyb = 0;
    % for z = 1:3
    %     err_z_h = Tz(z) - SP_hyb;
    %     if abs(err_z_h) < P.hybrid_db; err_z_h = 0; end
    %     err_z_h = max(0, err_z_h);
    % 
    %     deriv_h = (err_z_h - hyb_eprev(z)) / 3600;
    %     hyb_eprev(z) = err_z_h;
    % 
    %     zone_max_h = P.vrf_Qmax * P.zone_frac(z);
    %     Q_z_h = P.pid_Kp(z)*err_z_h ...
    %           + P.pid_Ki(z)*hyb_integ(z) ...
    %           + P.pid_Kd(z)*deriv_h;
    %     Q_z_h = max(0, min(zone_max_h, Q_z_h));
    % 
    %     if Q_z_h < zone_max_h*0.97 && err_z_h > 0
    %         hyb_integ(z) = clamp(hyb_integ(z)+err_z_h*3600, 0, P.pid_iLim(z));
    %     elseif err_z_h == 0
    %         hyb_integ(z) = hyb_integ(z) * 0.97;
    %     end
    % 
    %     if Q_z_h < 150; Q_z_h = 0; end
    %     Q_pid_hyb = Q_pid_hyb + Q_z_h;
    % end
    % Q_pid_hyb = min(Q_pid_hyb, P.vrf_Qmax * 0.95);
    % 
    % %% ── Layer 3: Adaptive Weight Update ───────────────
    % %% Record current performance of each component
    % hyb_mpc_err_hist(hyb_hist_ptr) = abs(T_now_hyb - SP_hyb);
    % hyb_pid_err_hist(hyb_hist_ptr) = abs(T_avg - SP_hyb);
    % hyb_hist_ptr = mod(hyb_hist_ptr, P.hybrid_mem) + 1;
    % hyb_step = hyb_step + 1;
    % 
    % if hyb_step > P.hybrid_mem
    %     %% Enough history — update weights based on recent performance
    %     mpc_recent = mean(hyb_mpc_err_hist) + 0.1;   % +0.1 avoids div/0
    %     pid_recent = mean(hyb_pid_err_hist) + 0.1;
    % 
    %     %% Lower recent error → higher weight (inverse error weighting)
    %     inv_m = 1 / mpc_recent;
    %     inv_p = 1 / pid_recent;
    %     inv_s = 0.15;   % fixed safety term weight
    %     total_inv = inv_m + inv_p + inv_s;
    % 
    %     alpha_new = [inv_m/total_inv; inv_p/total_inv; inv_s/total_inv];
    % 
    %     %% Exponential moving average (smooth weight transitions)
    %     hyb_alpha = (1 - P.hybrid_lr)*hyb_alpha + P.hybrid_lr*alpha_new;
    %     hyb_alpha = hyb_alpha / sum(hyb_alpha);   % normalize
    % 
    %     %% Context overrides ─────────────────────────
    %     %% Emergency: zone very hot AND MPC not helping → shift to PID
    %     if T_now_hyb > SP_hyb + 4.0 && mpc_recent > 3.0
    %         hyb_alpha(1) = max(0.20, hyb_alpha(1) - 0.10);
    %         hyb_alpha(2) = min(0.75, hyb_alpha(2) + 0.10);
    %         hyb_alpha(3) = 1 - hyb_alpha(1) - hyb_alpha(2);
    %     end
    % 
    %     %% Mild + off-peak: trust MPC more for pre-cooling
    %     if T_now_hyb < SP_hyb + 1.0 && trf <= 5.20
    %         hyb_alpha(1) = min(0.80, hyb_alpha(1) + 0.05);
    %         hyb_alpha(2) = max(0.15, hyb_alpha(2) - 0.05);
    %         hyb_alpha(3) = 1 - hyb_alpha(1) - hyb_alpha(2);
    %     end
    % 
    %     %% Monsoon season: trust PID more (humidity makes MPC less accurate)
    %     if sea == 3 && RH_o > 75
    %         hyb_alpha(1) = max(0.35, hyb_alpha(1) - 0.05);
    %         hyb_alpha(2) = min(0.60, hyb_alpha(2) + 0.05);
    %         hyb_alpha(3) = 1 - hyb_alpha(1) - hyb_alpha(2);
    %     end
    % end
    % 
    % %% ── Fusion: Weighted combination ──────────────────
    % err_total_hyb = max(0, T_avg - SP_hyb);
    % Q_safety_hyb  = min(err_total_hyb * 500, P.vrf_Qmax * 0.40);
    % 
    % Q_cool = hyb_alpha(1) * Q_mpc_hyb ...
    %        + hyb_alpha(2) * Q_pid_hyb ...
    %        + hyb_alpha(3) * Q_safety_hyb;
    % 
    % Q_cool = max(0, min(P.vrf_Qmax, Q_cool));
    % mode_log(i) = 3;
        end


        % Clamp to capacity
        Q_cool = max(0, min(P.vrf_Qmax*1.1, Q_cool));
        Q_cool_log(i) = Q_cool;

        % IEC assist (pre-monsoon)
        Q_iec = 0; P_iec = 0;
        if T_o > 28 && RH_o < P.iec_RH_max && Q_cool > 200
            Q_iec = iec_cap(i) * 0.5;
            P_iec = P.iec_Pfan;
            mode_log(i) = 2;
        end

        % DOAS
        doas_on = (RH_o > 65 && T_o > 22);
        P_doas  = doas_P(i) * 1000;   % W
        if doas_on && mode_log(i) == 3; mode_log(i) = 4; end
        if mode_log(i) == 0; mode_log(i) = 1; end

        % Electrical power — VRF
        COP_h = cop_vrf(T_o, P);
        P_vrf = 0;
        if Q_cool > 0 && T_o >= P.vrf_T_min && T_o <= P.vrf_T_max
            P_vrf = Q_cool / (1000 * COP_h);   % kW
        end

        % Zone thermal update (Euler integration)
        Q_hvac_z = -(Q_cool + Q_iec) * P.zone_frac';
        Tz = zone_euler(Tz, T_o, T_sa, i, Q_hvac_z, P, ...
                        Q_sol_z1, Q_sol_z2, Q_sol_z3, ...
                        Q_int_z1, Q_int_z2, Q_int_z3, 3600);

        % CO2
        occ_f  = occ_sched(h24);
        G_co2  = P.CO2_gen * occ_f;
        Q_vent = P.Q_inf_m3s + P.Q_oa_m3s*double(doas_on);
        dco2   = (G_co2 - Q_vent*(co2/1e6 - P.CO2_out/1e6)) / P.V_total;
        co2    = clamp(co2 + dco2*3600*1e6, P.CO2_out, 3000);

        % PM2.5
        cook_f = cook_sched(h24);
        G_pm   = P.G_PM_cook*cook_f + P.G_PM_bg;
        dpm    = (P.Q_inf_m3s*PM25_out(i) + G_pm*P.V_total ...
                  - P.Q_recirc*P.eta_HEPA*pm - P.lambda_dep*pm*P.V_total) / P.V_total;
        pm     = clamp(pm + dpm*3600, 0, 300);

        Tz1_log(i) = Tz(1); Tz2_log(i) = Tz(2); Tz3_log(i) = Tz(3);
        CO2_log(i) = co2;   PM_log(i)   = pm;
        Pvrf_log(i)= P_vrf; Piec_log(i) = P_iec/1000;
        Pdoas_log(i)= P_doas/1000;
    end

    % Store results
    E_vrf  = sum(Pvrf_log);
    E_iec  = sum(Piec_log);
    E_doas = sum(Pdoas_log);
    E_tot  = E_vrf + E_iec + E_doas;
    h_comf = sum(Tz2_log>=22 & Tz2_log<=26);
    h_cool = sum(Tz2_log>26);
    h_heat = sum(Tz2_log<22);
    rmse   = rms(Tz2_log - T_SET);

    RESULTS.(cid).Tz1     = Tz1_log;
    RESULTS.(cid).Tz2     = Tz2_log;
    RESULTS.(cid).Tz3     = Tz3_log;
    RESULTS.(cid).CO2     = CO2_log;
    RESULTS.(cid).PM25    = PM_log;
    RESULTS.(cid).E_vrf   = E_vrf;
    RESULTS.(cid).E_iec   = E_iec;
    RESULTS.(cid).E_doas  = E_doas;
    RESULTS.(cid).E_total = E_tot;
    RESULTS.(cid).h_comf  = h_comf;
    RESULTS.(cid).h_cool  = h_cool;
    RESULTS.(cid).h_heat  = h_heat;
    RESULTS.(cid).rmse    = rmse;
    RESULTS.(cid).mode    = mode_log;
    RESULTS.(cid).carbon  = E_tot * GRID_FACTOR;
    RESULTS.(cid).cost    = E_tot * 7.15;
    RESULTS.(cid).specific= E_tot / 95;
    
  
    fprintf(' %.1f s | E=%.0fkWh spec=%.1f Comfort=%.0f%% RMSE=%.2f°C\n',...
        toc(tic_c), E_tot, E_tot/95, h_comf/N_SIM*100, rmse);
end
  % Sanity check after baseline simulation
if RESULTS.baseline.h_comf/N_SIM*100 < 10
    fprintf('  WARNING: Baseline comfort < 10%% — controller may be malfunctioning\n');
    fprintf('  Check case baseline for persistent variables or logic errors\n');
end

save('All_Results.mat', 'RESULTS', 'P', 'TS');
fprintf('  Saved: All_Results.mat\n');

%% ────────────────────────────────────────────────────────────────
%%  STEP 6: PRINT COMPARISON TABLE
%% ────────────────────────────────────────────────────────────────
% fprintf('\n');
% fprintf('══════════════════════════════════════════════════════════════════\n');
% fprintf('  4-CONTROLLER COMPARISON TABLE\n');
% fprintf('══════════════════════════════════════════════════════════════════\n');
% fmt = '%-35s %10s %10s %10s %10s\n';
% fprintf(fmt,'KPI','Baseline','PID','MPC','RL-DQN');
% fprintf('%s\n', repmat('─',1,78));
% kpis = {
%     'Annual energy (kWh/yr)',        'E_total',  @(v) sprintf('%.0f',v);
%     'Specific (kWh/m²/yr)',          'specific', @(v) sprintf('%.1f',v);
%     'Comfort 22–26°C (%)',           'h_comf',   @(v) sprintf('%.1f',v/N_SIM*100);
%     'Cooling hours >26°C (h/yr)',    'h_cool',   @(v) sprintf('%d',round(v));
%     'Heating hours <22°C (h/yr)',    'h_heat',   @(v) sprintf('%d',round(v));
%     'RMSE from setpoint (°C)',       'rmse',     @(v) sprintf('%.2f',v);
%     'Carbon (kgCO₂/yr)',             'carbon',   @(v) sprintf('%.0f',v);
%     'Cost (₹/yr)',                   'cost',     @(v) sprintf('%.0f',v);
%     'CO₂ peak (ppm)',                'CO2',      @(v) sprintf('%.0f',max(v));
%     'PM₂.₅ mean (µg/m³)',           'PM25',     @(v) sprintf('%.2f',mean(v));
% };
% baseline_E = RESULTS.baseline.E_total;
% for r = 1:size(kpis,1)
%     row = kpis(r,:);
%     vals = cellfun(@(c) RESULTS.(c).(row{2}), ctrl_ids, 'UniformOutput',false);
%     strs = cellfun(@(v) row{3}(v), vals, 'UniformOutput',false);
%     fprintf(fmt, row{1}, strs{:});
% end
% 
% % fprintf('\nEnergy savings vs baseline:\n');
% % for ci = 2:4
% %     pct = (RESULTS.baseline.E_total - RESULTS.(ctrl_ids{ci}).E_total) / ...
% %            RESULTS.baseline.E_total * 100;
% %     fprintf('  %-8s : %+.1f%%\n', ctrl_names{ci}, pct);
% % end
% fprintf('\nEnergy savings vs baseline:\n');
% for ci = 2:5  % changed from 2:4 to 2:5
%     pct = (RESULTS.baseline.E_total - RESULTS.(ctrl_ids{ci}).E_total) / ...
%            RESULTS.baseline.E_total * 100;
%     fprintf('  %-8s : %+.1f%%\n', ctrl_names{ci}, pct);
% end

%% ────────────────────────────────────────────────────────────────
%%  STEP 6: PRINT COMPARISON TABLE
%% ────────────────────────────────────────────────────────────────

fprintf('\n');

fprintf('════════════════════════════════════════════════════════════════════════════════════════════\n');
fprintf('                           5-CONTROLLER COMPARISON TABLE\n');
fprintf('════════════════════════════════════════════════════════════════════════════════════════════\n');

fprintf('%-35s %-10s %-10s %-10s %-10s %-10s\n', ...
    'KPI', 'Baseline', 'PID', 'MPC', 'RL-DQN', 'Hybrid');

fprintf('%s\n', repmat('─',1,92));

%% Annual Energy
fprintf('%-35s %-10.0f %-10.0f %-10.0f %-10.0f %-10.0f\n', ...
    'Annual Energy (kWh/yr)', ...
    RESULTS.baseline.E_total,...
    RESULTS.pid.E_total,...
    RESULTS.mpc.E_total,...
    RESULTS.rl.E_total,...
    RESULTS.hybrid.E_total);

%% Specific Energy
fprintf('%-35s %-10.1f %-10.1f %-10.1f %-10.1f %-10.1f\n', ...
    'Specific Energy (kWh/m²/yr)', ...
    RESULTS.baseline.specific,...
    RESULTS.pid.specific,...
    RESULTS.mpc.specific,...
    RESULTS.rl.specific,...
    RESULTS.hybrid.specific);

%% Comfort
fprintf('%-35s %-10.1f %-10.1f %-10.1f %-10.1f %-10.1f\n', ...
    'Comfort 22–26°C (%)', ...
    RESULTS.baseline.h_comf/N_SIM*100,...
    RESULTS.pid.h_comf/N_SIM*100,...
    RESULTS.mpc.h_comf/N_SIM*100,...
    RESULTS.rl.h_comf/N_SIM*100,...
    RESULTS.hybrid.h_comf/N_SIM*100);

%% Cooling Hours
fprintf('%-35s %-10d %-10d %-10d %-10d %-10d\n', ...
    'Cooling Hours >26°C (h/yr)', ...
    round(RESULTS.baseline.h_cool),...
    round(RESULTS.pid.h_cool),...
    round(RESULTS.mpc.h_cool),...
    round(RESULTS.rl.h_cool),...
    round(RESULTS.hybrid.h_cool));

%% Heating Hours
fprintf('%-35s %-10d %-10d %-10d %-10d %-10d\n', ...
    'Heating Hours <22°C (h/yr)', ...
    round(RESULTS.baseline.h_heat),...
    round(RESULTS.pid.h_heat),...
    round(RESULTS.mpc.h_heat),...
    round(RESULTS.rl.h_heat),...
    round(RESULTS.hybrid.h_heat));

%% RMSE
fprintf('%-35s %-10.2f %-10.2f %-10.2f %-10.2f %-10.2f\n', ...
    'RMSE from Setpoint (°C)', ...
    RESULTS.baseline.rmse,...
    RESULTS.pid.rmse,...
    RESULTS.mpc.rmse,...
    RESULTS.rl.rmse,...
    RESULTS.hybrid.rmse);

%% Carbon
fprintf('%-35s %-10.0f %-10.0f %-10.0f %-10.0f %-10.0f\n', ...
    'Carbon Emissions (kgCO₂/yr)', ...
    RESULTS.baseline.carbon,...
    RESULTS.pid.carbon,...
    RESULTS.mpc.carbon,...
    RESULTS.rl.carbon,...
    RESULTS.hybrid.carbon);

%% Cost
fprintf('%-35s %-10.0f %-10.0f %-10.0f %-10.0f %-10.0f\n', ...
    'Annual Cost (₹/yr)', ...
    RESULTS.baseline.cost,...
    RESULTS.pid.cost,...
    RESULTS.mpc.cost,...
    RESULTS.rl.cost,...
    RESULTS.hybrid.cost);

%% CO2 Peak
fprintf('%-35s %-10.0f %-10.0f %-10.0f %-10.0f %-10.0f\n', ...
    'CO₂ Peak (ppm)', ...
    max(RESULTS.baseline.CO2),...
    max(RESULTS.pid.CO2),...
    max(RESULTS.mpc.CO2),...
    max(RESULTS.rl.CO2),...
    max(RESULTS.hybrid.CO2));

%% PM2.5
fprintf('%-35s %-10.2f %-10.2f %-10.2f %-10.2f %-10.2f\n', ...
    'PM₂.₅ Mean (µg/m³)', ...
    mean(RESULTS.baseline.PM25),...
    mean(RESULTS.pid.PM25),...
    mean(RESULTS.mpc.PM25),...
    mean(RESULTS.rl.PM25),...
    mean(RESULTS.hybrid.PM25));

fprintf('════════════════════════════════════════════════════════════════════════════════════════════\n');

%% Energy Savings
fprintf('\nEnergy savings vs baseline:\n');

for ci = 2:5
    pct = (RESULTS.baseline.E_total - RESULTS.(ctrl_ids{ci}).E_total) ...
          / RESULTS.baseline.E_total * 100;

    fprintf('  %-10s : %+6.1f%%\n', ctrl_names{ci}, pct);
end

%% ────────────────────────────────────────────────────────────────
%%  STEP 7: BUILD SIMULINK MODEL
%% ────────────────────────────────────────────────────────────────
if BUILD_SIMULINK
    fprintf('\nSTEP 7: Building Simulink model ...\n');
    build_simulink_model(P, TS, N_SIM, T_SET, nn_best_W, nn_best_b);
end

%% ────────────────────────────────────────────────────────────────
%%  STEP 8: GENERATE FIGURES
%% ────────────────────────────────────────────────────────────────
if PLOT_RESULTS
    fprintf('\nSTEP 8: Generating figures\n');
    generate_figures(RESULTS, PuneTMY, T_db, N_SIM, T_SET, P, tod_full, ...
                     ep_R, ep_C, n_ep, ctrl_names, ctrl_ids, GRID_FACTOR);
end

fprintf('\n════════════════════════════════════════════════════════════════\n');
fprintf('  COMPLETE.  Files generated:\n');
fprintf('    Pune_HVAC_Complete.slx  — Simulink model\n');
fprintf('    All_Results.mat         — All controller results\n');
fprintf('    RL_Network_Weights.mat  — Trained DQN weights\n');
fprintf('    SimWS_Full.mat          — Simulink workspace data\n');
fprintf('    Fig01–Fig08  .png       — Publication figures\n');
fprintf('════════════════════════════════════════════════════════════════\n');


%% ================================================================
%%  BUILD SIMULINK MODEL FUNCTION
%% ================================================================
function build_simulink_model(P, TS, N_SIM, T_SET, RL_W, RL_b)

mdl = 'Pune_HVAC_Complete';
stop_t = num2str((N_SIM-1)*3600);

% ── Close / delete old model
if bdIsLoaded(mdl); close_system(mdl, 0); end
if exist([mdl '.slx'],'file'); delete([mdl '.slx']); end

new_system(mdl);
open_system(mdl);

% ── Solver
set_param(mdl,...
    'SolverType',       'Variable-step',...
    'Solver',           'ode23t',...
    'MaxStep',          '3600',...
    'MinStep',          'auto',...
    'StopTime',         stop_t,...
    'SaveTime',         'on',...
    'TimeSaveName',     'tout',...
    'SaveOutput',       'on',...
    'OutputSaveName',   'yout',...
    'SignalLogging',    'on',...
    'SignalLoggingName','logsout',...
    'SaveFormat',       'Dataset');

% ── Model workspace
mws = get_param(mdl,'ModelWorkspace');
mws.DataSource = 'MAT-File';
mws.FileName   = 'SimWS_Full';
mws.reload();

assign_mws(mws, 'P', P);
for z = 1:3
    assignin(mws, sprintf('C%d',z),      P.C(z));
    assignin(mws, sprintf('UA_wall%d',z),P.UA_wall(z));
    assignin(mws, sprintf('UA_roof%d',z),P.UA_roof(z));
    assignin(mws, sprintf('UA_glz%d',z), P.UA_glz(z));
    assignin(mws, sprintf('UA_inf%d',z), P.UA_inf(z));
    assignin(mws, sprintf('UA_tot%d',z), P.UA_tot(z));
end
assignin(mws,'R_z1z2',  P.R_z1z2);
assignin(mws,'R_z2z3',  P.R_z2z3);
assignin(mws,'T_init',  26);
assignin(mws,'T_SET',   T_SET);
assignin(mws,'vrf_Qmax',P.vrf_Qmax);
assignin(mws,'vrf_Qmin',P.vrf_Qmin);
assignin(mws,'vrf_COP', P.vrf_COP);
assignin(mws,'vrf_T_ref',P.vrf_T_ref);
assignin(mws,'vrf_cop_a',P.vrf_cop_a);
assignin(mws,'vrf_cop_b',P.vrf_cop_b);
assignin(mws,'vrf_T_min',P.vrf_T_min);
assignin(mws,'vrf_T_max',P.vrf_T_max);
assignin(mws,'pid_Kp',  P.pid_Kp);
assignin(mws,'pid_Ki',  P.pid_Ki);
assignin(mws,'pid_Kd',  P.pid_Kd);
assignin(mws,'pid_iLim',P.pid_iLim);
assignin(mws,'pid_db',  P.pid_db);
assignin(mws,'mpc_Hp',  P.mpc_Hp);
assignin(mws,'mpc_Hc',  P.mpc_Hc);
assignin(mws,'mpc_w_c', P.mpc_w_c);
assignin(mws,'mpc_w_e', P.mpc_w_e);
assignin(mws,'mpc_w_r', P.mpc_w_r);
assignin(mws,'mpc_a',   P.mpc_a);
assignin(mws,'mpc_b',   P.mpc_b);
assignin(mws,'rl_actions',P.rl_actions);
assignin(mws,'rl_gamma', P.rl_gamma);
assignin(mws,'RL_W',    RL_W);
assignin(mws,'RL_b',    RL_b);
% CRITICAL: Also store individual weight matrices so Constant blocks
% and MATLAB Function blocks can access them without cell indexing
assignin(mws,'W1', RL_W{1});
assignin(mws,'W2', RL_W{2});
assignin(mws,'W3', RL_W{3});
assignin(mws,'b1', RL_b{1});
assignin(mws,'b2', RL_b{2});
assignin(mws,'b3', RL_b{3});
assignin(mws,'zone_frac',P.zone_frac);
assignin(mws,'V_total', P.V_total);
assignin(mws,'Q_inf',   P.Q_inf_m3s);
assignin(mws,'Q_oa',    P.Q_oa_m3s);
assignin(mws,'CO2_out', P.CO2_out);
assignin(mws,'CO2_gen', P.CO2_gen);
assignin(mws,'eta_HEPA',P.eta_HEPA);
assignin(mws,'lambda_dep',P.lambda_dep);
assignin(mws,'G_PM_cook',P.G_PM_cook);
assignin(mws,'G_PM_bg', P.G_PM_bg);
assignin(mws,'Q_recirc',P.Q_recirc);
assignin(mws,'iec_Pfan',P.iec_Pfan);

fprintf('  Model workspace assigned.\n');

% ── Add From Workspace blocks (weather inputs)
fw_defs = {
    % name              variable            px    py
    'FW_T_outdoor',   'TS.T_outdoor',       30,   30;
    'FW_T_solair',    'TS.T_sol_air',       30,  100;
    'FW_RH',          'TS.RH_outdoor',      30,  170;
    'FW_GHI',         'TS.GHI',             30,  240;
    'FW_T_wb',        'TS.T_wb',            30,  310;
    'FW_season',      'TS.season',          30,  380;
    'FW_tariff',      'TS.tod_tariff',      30,  450;
    'FW_Qsol1',       'TS.Q_sol_z1',        30,  540;
    'FW_Qsol2',       'TS.Q_sol_z2',        30,  610;
    'FW_Qsol3',       'TS.Q_sol_z3',        30,  680;
    'FW_Qint1',       'TS.Q_int_z1',        30,  750;
    'FW_Qint2',       'TS.Q_int_z2',        30,  820;
    'FW_Qint3',       'TS.Q_int_z3',        30,  890;
    'FW_iec_cap',     'TS.iec_cap',         30,  960;
    'FW_doas_P',      'TS.doas_P',          30, 1030;
    'FW_PM25_out',    'TS.PM25_out',        30, 1100;
};

% Add Constant blocks for RL weight matrices using NUMERIC values
% mat2str embeds the actual matrix numbers — no workspace lookup needed
wt_names = {'W1','W2','W3','b1','b2','b3'};
wt_vals  = {RL_W{1}, RL_W{2}, RL_W{3}, RL_b{1}, RL_b{2}, RL_b{3}};
for wi = 1:6
    add_block('simulink/Sources/Constant',...
        [mdl '/WtConst_' wt_names{wi}],...
        'Value',    mat2str(wt_vals{wi}, 8),...  % embed numbers directly
        'Position', [230, 1200+(wi-1)*55, 370, 1230+(wi-1)*55]);
end

% Signals that should be interpolated (continuous: temperatures)
interp_on  = {'FW_T_outdoor','FW_T_solair','FW_RH','FW_GHI','FW_T_wb'};
% Signals that are discrete/stepped (no interpolation needed)
interp_off = {'FW_season','FW_tariff','FW_Qsol1','FW_Qsol2','FW_Qsol3',...
              'FW_Qint1','FW_Qint2','FW_Qint3','FW_iec_cap','FW_doas_P','FW_PM25_out'};

for r = 1:size(fw_defs,1)
    nm  = fw_defs{r,1};
    var = fw_defs{r,2};
    px  = fw_defs{r,3};
    py  = fw_defs{r,4};

    % Determine interpolation setting for this block
    if any(strcmp(nm, interp_on))
        interp_str = 'on';
    else
        interp_str = 'off';
    end

    add_block('simulink/Sources/From Workspace',...
        [mdl '/' nm],...
        'VariableName',           var,...
        'SampleTime',             '-1',...
        'Interpolate',            interp_str,...
        'OutputAfterFinalValue',  'Extrapolation',...
        'Position',               [px py px+140 py+30]);
end
fprintf('  Weather input blocks added (all with Extrapolation after final value).\n');

% ── Build 4 Controller subsystems + 3 Thermal Zone subsystems
build_weather_preprocessor(mdl);
build_zone_subsystem(mdl, 1, 380, 300,  P);
build_zone_subsystem(mdl, 2, 380, 620,  P);
build_zone_subsystem(mdl, 3, 380, 940,  P);
build_baseline_ctrl(mdl, 750, 300);
build_pid_ctrl(mdl, 750, 600);
build_mpc_ctrl(mdl, 750, 900);
build_rl_ctrl(mdl, 750, 1200);
build_hybrid_ctrl(mdl, 750, 1550, P);
build_hvac_equipment(mdl, 1100, 600);
build_iaq_subsystem(mdl, 1450, 600);
build_output_logger(mdl, 1800, 600);
build_controller_mux(mdl, 1000, 750);

% ── Top-level wiring
wire_model(mdl);

% ── Annotations
add_annotation(mdl, 'Pune 3BHK HVAC — Complete Multi-Controller Model',...
    [100 20 800 50]);
add_annotation(mdl, 'Zone 1: Master Bedroom | Zone 2: Living+Dining | Zone 3: 2nd Bedroom',...
    [380 265 780 285]);

save_system(mdl);
fprintf('  Saved: %s.slx\n', mdl);
fprintf('\n  ┌─────────────────────────────────────────────┐\n');
fprintf('  │  To simulate:  sim(''%s'')   │\n', mdl);
fprintf('  │  Or press Run in Simulink.                  │\n');
fprintf('  └─────────────────────────────────────────────┘\n');
end

%% ================================================================
function build_weather_preprocessor(mdl)
% Compute T_sol_air from T_outdoor + GHI inside Simulink
ss = [mdl '/WeatherPreprocessor'];
add_block('simulink/Ports & Subsystems/Subsystem', ss,...
    'Position',[230 40 350 350]);
clean_subsystem(ss);

add_inport(ss,'T_outdoor',1, [20 30]);
add_inport(ss,'GHI',      2, [20 90]);
add_inport(ss,'RH',       3, [20 150]);
add_inport(ss,'T_wb',     4, [20 210]);

% T_sol_air = T_outdoor + (0.7*GHI - 0.9*63)/22.7
add_block('simulink/Math Operations/Gain',[ss '/Gain_alpha'],...
    'Gain','0.7','Position',[100 80 130 100]);
add_block('simulink/Sources/Constant',[ss '/eps_dR'],...
    'Value','0.9*63','Position',[100 120 160 140]);
add_block('simulink/Math Operations/Sum',[ss '/Sum_sol'],...
    'Inputs','+-','Position',[180 88 210 118]);
add_block('simulink/Math Operations/Gain',[ss '/Gain_h'],...
    'Gain','1/22.7','Position',[230 88 270 118]);
add_block('simulink/Math Operations/Sum',[ss '/Sum_Tsol'],...
    'Inputs','++','Position',[310 25 340 55]);

add_outport(ss,'T_sol_air',1,[380 35]);
add_outport(ss,'T_outdoor_pass',2,[380 95]);
add_outport(ss,'RH_pass',3,[380 155]);
add_outport(ss,'T_wb_pass',4,[380 215]);

try
    add_line(ss,'T_outdoor/1','Sum_Tsol/1','autorouting','on');
    add_line(ss,'GHI/1','Gain_alpha/1','autorouting','on');
    add_line(ss,'Gain_alpha/1','Sum_sol/1','autorouting','on');
    add_line(ss,'eps_dR/1','Sum_sol/2','autorouting','on');
    add_line(ss,'Sum_sol/1','Gain_h/1','autorouting','on');
    add_line(ss,'Gain_h/1','Sum_Tsol/2','autorouting','on');
    add_line(ss,'Sum_Tsol/1','T_sol_air/1','autorouting','on');
    add_line(ss,'T_outdoor/1','T_outdoor_pass/1','autorouting','on');
    add_line(ss,'RH/1','RH_pass/1','autorouting','on');
    add_line(ss,'T_wb/1','T_wb_pass/1','autorouting','on');
catch; end
end

%% ================================================================
function build_zone_subsystem(mdl, zone_idx, xpos, ypos, P)
% RC thermal zone subsystem — ALL GAINS USE NUMERIC VALUES (no workspace lookup)
% Inputs: T_outdoor, T_sol_air, Q_solar, Q_int, Q_hvac, T_adj1, T_adj2
% Output: T_zone

names = {'MasterBed','LivingDining','SecondBed'};
ss = [mdl '/Zone' num2str(zone_idx) '_' names{zone_idx}];
add_block('simulink/Ports & Subsystems/Subsystem', ss,...
    'Position',[xpos ypos xpos+160 ypos+260]);
clean_subsystem(ss);

% Inports
ports = {'T_outdoor','T_sol_air','Q_solar','Q_int','Q_hvac',...
         'T_adj1','T_adj2'};
for p = 1:length(ports)
    add_inport(ss, ports{p}, p, [20, 20+p*35]);
end

% Outport
add_outport(ss,'T_zone',1,[560 140]);

% NUMERIC UA values — no variable name strings, no workspace dependency
UA_vals = [P.UA_wall(zone_idx), P.UA_roof(zone_idx), ...
           P.UA_glz(zone_idx),  P.UA_inf(zone_idx)];
src_in  = {'T_outdoor','T_sol_air','T_outdoor','T_outdoor'};

for q = 1:length(UA_vals)
    add_block('simulink/Math Operations/Sum',...
        [ss '/Sub_' num2str(q)],...
        'Inputs','+-','Position',[120, 30+q*50, 145, 60+q*50]);
    add_block('simulink/Math Operations/Gain',...
        [ss '/UA_' num2str(q)],...
        'Gain', num2str(UA_vals(q), '%.6g'),...   % NUMERIC — not a variable name
        'Position',[160, 30+q*50, 210, 60+q*50]);
end

% Inter-zone conductance values — NUMERIC
% Zone topology: Z1 <--R_z1z2--> Z2 <--R_z2z3--> Z3
if zone_idx == 1
    iz1_cond = 1/P.R_z1z2;   % W/K conductance to Z2
    iz2_cond = 0;             % Z1 not adjacent to Z3 (gain=0, net Q=0)
elseif zone_idx == 2
    iz1_cond = 1/P.R_z1z2;   % conductance to Z1
    iz2_cond = 1/P.R_z2z3;   % conductance to Z3
else
    iz1_cond = 1/P.R_z2z3;   % conductance to Z2
    iz2_cond = 0;             % Z3 not adjacent to Z1
end

add_block('simulink/Math Operations/Sum',...
    [ss '/Sub_iz1'],'Inputs','+-','Position',[120, 260, 145, 290]);
add_block('simulink/Math Operations/Gain',...
    [ss '/G_iz1'],'Gain', num2str(iz1_cond, '%.6g'),...
    'Position',[160, 260, 210, 290]);

add_block('simulink/Math Operations/Sum',...
    [ss '/Sub_iz2'],'Inputs','+-','Position',[120, 310, 145, 340]);
add_block('simulink/Math Operations/Gain',...
    [ss '/G_iz2'],'Gain', num2str(iz2_cond, '%.6g'),...
    'Position',[160, 310, 210, 340]);

% Master sum and 1/C gain — NUMERIC
add_block('simulink/Math Operations/Sum',...
    [ss '/Q_net'],'Inputs','+++++++++',...
    'Position',[280, 100, 310, 380]);

add_block('simulink/Math Operations/Gain',...
    [ss '/invC'],'Gain', num2str(1/P.C(zone_idx), '%.6g'),... % NUMERIC
    'Position',[340, 220, 395, 260]);

% Integrator with numeric initial condition
add_block('simulink/Continuous/Integrator',...
    [ss '/Intg'],...
    'InitialCondition','26',...   % numeric string — not a variable
    'UpperSaturationLimit','80',...
    'LowerSaturationLimit','-5',...
    'Position',[430, 220, 490, 260]);

% Wiring inside zone subsystem
try
for q = 1:length(UA_vals)
        add_line(ss,[src_in{q} '/1'],['Sub_' num2str(q) '/1'],'autorouting','on');
        add_line(ss,'Intg/1',['Sub_' num2str(q) '/2'],'autorouting','on');
        add_line(ss,['Sub_' num2str(q) '/1'],['UA_' num2str(q) '/1'],'autorouting','on');
        add_line(ss,['UA_' num2str(q) '/1'],['Q_net/' num2str(q)],'autorouting','on');
    end
    add_line(ss,'T_adj1/1','Sub_iz1/1','autorouting','on');
    add_line(ss,'Intg/1',  'Sub_iz1/2','autorouting','on');
    add_line(ss,'Sub_iz1/1','G_iz1/1','autorouting','on');
    add_line(ss,'G_iz1/1',  'Q_net/5','autorouting','on');
    add_line(ss,'T_adj2/1','Sub_iz2/1','autorouting','on');
    add_line(ss,'Intg/1',  'Sub_iz2/2','autorouting','on');
    add_line(ss,'Sub_iz2/1','G_iz2/1','autorouting','on');
    add_line(ss,'G_iz2/1',  'Q_net/6','autorouting','on');
    add_line(ss,'Q_solar/1','Q_net/7','autorouting','on');
    add_line(ss,'Q_int/1',  'Q_net/8','autorouting','on');
    add_line(ss,'Q_hvac/1', 'Q_net/9','autorouting','on');
    add_line(ss,'Q_net/1',  'invC/1','autorouting','on');
    add_line(ss,'invC/1',   'Intg/1','autorouting','on');
    add_line(ss,'Intg/1',   'T_zone/1','autorouting','on');
catch; end
end

%% ================================================================
function build_baseline_ctrl(mdl, xpos, ypos)
% Baseline rule-based controller
ss = [mdl '/Ctrl_Baseline'];
add_block('simulink/Ports & Subsystems/Subsystem', ss,...
    'Position',[xpos ypos xpos+160 ypos+120]);
clean_subsystem(ss);
add_inport(ss,'T_zone_avg',1,[20 40]);
add_inport(ss,'T_outdoor',  2,[20 80]);
add_inport(ss,'RH',         3,[20 120]);
add_outport(ss,'Q_cool',1,[400 80]);

% MATLAB Function for rule-based logic
mfb = [ss '/BaselineMF'];
add_block('simulink/User-Defined Functions/MATLAB Function', mfb,...
    'Position',[100 30 340 150]);

code = [...
'function Q_cool = baseline_ctrl(T_avg, T_out, RH_out)\n'...
'%#codegen\n'...
'vrf_Qmax = 11000; T_SET = 24;\n'...
'SP = T_SET;\n'...
'Q_cool = 0;\n'...
'%% Only cool if zone is actually above setpoint\n'...
'if T_avg <= SP; return; end\n'...
'dT = T_avg - SP;\n'...
'if T_out > 35\n'...
'    Q_cool = vrf_Qmax * min(1.0, max(0.2, dT/3));\n'...
'elseif T_out > 28 && RH_out < 50\n'...
'    Q_cool = vrf_Qmax * min(0.6, max(0.2, dT/4));\n'...
'elseif RH_out > 65 && T_out > 22\n'...
'    Q_cool = vrf_Qmax * min(0.9, max(0.2, dT/3));\n'...
'elseif T_avg > SP + 0.5\n'...
'    Q_cool = vrf_Qmax * min(0.7, max(0.2, dT/5));\n'...
'end\n'...
'Q_cool = max(0, min(vrf_Qmax, Q_cool));\n'...
'end\n'];

set_mf_code(mfb, code);
try
    add_line(ss,'T_zone_avg/1','BaselineMF/1','autorouting','on');
    add_line(ss,'T_outdoor/1', 'BaselineMF/2','autorouting','on');
    add_line(ss,'RH/1',        'BaselineMF/3','autorouting','on');
    add_line(ss,'BaselineMF/1','Q_cool/1','autorouting','on');
catch; end
end

%% ================================================================
function build_pid_ctrl(mdl, xpos, ypos)
% PID controller subsystem (3 zones, anti-windup, night setback)
ss = [mdl '/Ctrl_PID'];
add_block('simulink/Ports & Subsystems/Subsystem', ss,...
    'Position',[xpos ypos xpos+160 ypos+200]);
clean_subsystem(ss);

% Inports: T_z1, T_z2, T_z3, time_of_day
    lbl = {'T_zone1','T_zone2','T_zone3','hour_of_day'};
    for p = 1:length(lbl)

    add_inport(ss, lbl{p}, p, [20, 20+p*45]);
end
add_outport(ss,'Q_cool',1,[500 100]);

mfb = [ss '/PID_MF'];
add_block('simulink/User-Defined Functions/MATLAB Function', mfb,...
    'Position',[80 20 460 220]);

% code = [...
% 'function Q_cool = pid_ctrl(Tz1, Tz2, Tz3, h24)\n'...
% '%#codegen\n'...
% 'persistent integ e_prev;\n'...
% 'if isempty(integ)\n'...
% '    integ = zeros(3,1); e_prev = zeros(3,1);\n'...
% 'end\n'...
% 'T_SET = 24; dt = 3600;\n'...
% 'Kp   = [150, 250, 100];\n'...
% 'Ki   = Kp / 7200;\n'...
% 'Kd   = Kp * 900;\n'...
% 'iLim = [5000, 8000, 4000];\n'...
% 'frac = [0.316, 0.474, 0.210];\n'...
% 'db   = 0.5;\n'...
% 'is_night = (h24 >= 23 || h24 <= 6);\n'...
% 'SP = T_SET + 2*double(is_night);\n'...
% 'Tz = [Tz1, Tz2, Tz3];\n'...
% 'Q_cool = 0;\n'...
% 'for z = 1:3\n'...
% '    err = Tz(z) - SP;\n'...
% '    if err < db; err = 0; end\n'...   % only cool, apply deadband
% '    integ(z) = max(-iLim(z), min(iLim(z), integ(z)+err*dt));\n'...
% '    deriv    = (err - e_prev(z)) / dt;\n'...
% '    e_prev(z)= err;\n'...
% '    zone_max = 11000 * frac(z) * 1.1;\n'...
% '    Q_z = Kp(z)*err + Ki(z)*integ(z) + Kd(z)*deriv;\n'...
% '    Q_z = max(0, min(zone_max, Q_z));\n'...
% '    if Q_z < 500; Q_z = 0; integ(z) = integ(z) - err*dt; end\n'... % anti-windup
% '    Q_cool = Q_cool + Q_z;\n'...
% 'end\n'...
% 'Q_cool = max(0, min(11000, Q_cool));\n'...
% 'end\n'];

code = [...
'function Q_cool = pid_ctrl(Tz1, Tz2, Tz3, h24)\n'...
'%#codegen\n'...
'persistent integ e_prev;\n'...
'if isempty(integ)\n'...
'    integ = zeros(3,1); e_prev = zeros(3,1);\n'...
'end\n'...
'T_SET = 24; dt = 3600;\n'...
'Kp   = [180, 300, 120];\n'...
'Ki   = Kp ./ 2400;\n'...
'Kd   = Kp .* 120;\n'...
'iLim = [3000, 5000, 2000];\n'...
'frac = [0.316, 0.474, 0.210];\n'...
'db   = 0.4;\n'...
'is_night = (h24 >= 23 || h24 <= 6);\n'...
'SP = T_SET + 2*double(is_night);\n'...
'Tz = [Tz1, Tz2, Tz3];\n'...
'Q_cool = 0;\n'...
'for z = 1:3\n'...
'    err = max(0, Tz(z) - SP);\n'...
'    if err < db; err = 0; end\n'...
'    deriv = (err - e_prev(z)) / dt;\n'...
'    e_prev(z) = err;\n'...
'    zone_max = 11000 * frac(z);\n'...
'    Q_z = Kp(z)*err + Ki(z)*integ(z) + Kd(z)*deriv;\n'...
'    Q_z = max(0, min(zone_max, Q_z));\n'...
'    if Q_z < zone_max*0.98 && err > 0\n'...
'        integ(z) = max(0, min(iLim(z), integ(z)+err*dt));\n'...
'    end\n'...
'    if Q_z < 200; Q_z = 0; end\n'...
'    Q_cool = Q_cool + Q_z;\n'...
'end\n'...
'Q_cool = max(0, min(11000, Q_cool));\n'...
'end\n'];
set_mf_code(mfb, code);
try
    add_line(ss,'T_zone1/1',    'PID_MF/1','autorouting','on');
    add_line(ss,'T_zone2/1',    'PID_MF/2','autorouting','on');
    add_line(ss,'T_zone3/1',    'PID_MF/3','autorouting','on');
    add_line(ss,'hour_of_day/1','PID_MF/4','autorouting','on');
    add_line(ss,'PID_MF/1',     'Q_cool/1','autorouting','on');
catch; end
end

%% ================================================================
function build_mpc_ctrl(mdl, xpos, ypos)
% MPC controller subsystem — simplified QP via gradient
ss = [mdl '/Ctrl_MPC'];
add_block('simulink/Ports & Subsystems/Subsystem', ss,...
    'Position',[xpos ypos xpos+160 ypos+220]);
clean_subsystem(ss);

% Inports: T_z2, T_outdoor_preview(scalar), tod_tariff, hour_of_day
lbl = {'T_zone2','T_outdoor','tod_tariff','hour_of_day'};
for p = 1:length(lbl)
    add_inport(ss, lbl{p}, p, [20, 20+p*50]);
end
add_outport(ss,'Q_cool',1,[500 120]);

mfb = [ss '/MPC_MF'];
add_block('simulink/User-Defined Functions/MATLAB Function', mfb,...
    'Position',[80 20 460 240]);

code = [...
'function Q_cool = mpc_ctrl(Tz2, T_out, tariff, h24)\n'...
'%#codegen\n'...
'persistent u_prev;\n'...
'if isempty(u_prev); u_prev = 0; end\n'...
'T_SET = 24; Hp = 12; vrf_Qmax = 11000; dt = 3600;\n'...
'is_night = (h24 >= 23 || h24 <= 6);\n'...
'SP = T_SET + 2*double(is_night);\n'...
'\n'...
'%% Gate: only cool if above setpoint + deadband\n'...
'if Tz2 <= SP + 0.3\n'...
'    Q_cool = 0; u_prev = 0; return;\n'...
'end\n'...
'\n'...
'%% Simplified prediction model (discrete first-order zone)\n'...
'%% a = exp(-dt/tau), tau ~ 6h → a ~ 0.846\n'...
'a = 0.85;\n'...
'sum_e = 0; xk = Tz2;\n'...
'for k = 1:Hp\n'...
'    xk = a*xk + (1-a)*(T_out + 2.0);\n' ... % disturbance estimate
'    w_k = power(0.95, k-1);\n'... % exponential decay weight
'    sum_e = sum_e + max(0, xk - SP) * w_k;\n'... % only penalise overheating
'end\n'...
'\n'...
'%% ToD energy weight: reduce cooling during peak tariff\n'...
'tod_w = 0.3 + 0.2*(tariff/7.15);\n'...
'\n'...
'%% Proportional demand from prediction error\n'...
'Q_demand = 800 * sum_e / Hp;\n'...  % 800 W/°C proportional gain
'\n'...
'%% Hard limits: 20%-110% of rated capacity\n'...
'Q_demand = max(vrf_Qmax * 0.20, min(vrf_Qmax * 1.05, Q_demand));\n'...
'\n'...
'%% Emergency cooling if very hot\n'...
'if Tz2 > SP + 3.0\n'...
'    Q_demand = vrf_Qmax;\n'...
'end\n'...
'\n'...
'%% Rate limit: max 30% change per hour\n'...
'du = Q_demand - u_prev;\n'...
'if abs(du) > vrf_Qmax * 0.30\n'...
'    Q_demand = u_prev + sign(du) * vrf_Qmax * 0.30;\n'...
'end\n'...
'\n'...
'Q_cool = max(0, min(vrf_Qmax * 1.05, Q_demand));\n'...
'u_prev = Q_cool;\n'...
'end\n'];

set_mf_code(mfb, code);
try
    add_line(ss,'T_zone2/1',     'MPC_MF/1','autorouting','on');
    add_line(ss,'T_outdoor/1',   'MPC_MF/2','autorouting','on');
    add_line(ss,'tod_tariff/1',  'MPC_MF/3','autorouting','on');
    add_line(ss,'hour_of_day/1', 'MPC_MF/4','autorouting','on');
    add_line(ss,'MPC_MF/1',      'Q_cool/1','autorouting','on');
catch; end
end

%% ================================================================
function build_rl_ctrl(mdl, xpos, ypos)
% RL-DQN controller subsystem
% Weights W1,W2,W3,b1,b2,b3 are flat matrices in the MODEL workspace
% They are passed as DATA STORE READ blocks → inputs to MATLAB Function
% (MATLAB Function blocks CANNOT index cell arrays — must be plain matrices)
ss = [mdl '/Ctrl_RL_DQN'];
add_block('simulink/Ports & Subsystems/Subsystem', ss,...
    'Position',[xpos ypos xpos+160 ypos+340]);
clean_subsystem(ss);

% Signal inports (6)
sig_ports = {'T_zone2','T_outdoor','RH','hour_of_day','season','tod_tariff'};
for p = 1:length(sig_ports)
    add_inport(ss, sig_ports{p}, p, [20, 10+p*42]);
end

% Weight inports (6) — W1 W2 W3 b1 b2 b3 fed from From Workspace at top level
wt_ports = {'W1_in','W2_in','W3_in','b1_in','b2_in','b3_in'};
for p = 1:length(wt_ports)
    add_inport(ss, wt_ports{p}, length(sig_ports)+p, [20, 275+p*38]);
end

add_outport(ss,'Q_cool',1,[580 200]);

mfb = [ss '/RL_DQN_MF'];
add_block('simulink/User-Defined Functions/MATLAB Function', mfb,...
    'Position',[120 10 540 510]);

% Weight matrices passed as direct function arguments — no cell array indexing
code = [...
'function Q_cool = rl_dqn_ctrl(Tz2,T_out,RH,h24,season,tariff,'...
'W1,W2,W3,b1,b2,b3)\n'...
'%#codegen\n'...
'persistent u_prev;\n'...
'if isempty(u_prev); u_prev = 0; end\n'...
'T_SET = 24; vrf_Qmax = 11000;\n'...
'actions = vrf_Qmax * [0, 0.15, 0.30, 0.50, 0.75, 1.00];\n'...
'\n'...
'%% State vector (8 features)\n'...
's = [(Tz2-24)/8; (T_out-28)/12; (RH-60)/25;\n'...
'     sin(2*pi*h24/24); cos(2*pi*h24/24);\n'...
'     (season-2.5)/1.5; (tariff-7.15)/1.8;\n'...
'     (Tz2-24)/6];\n'...
'\n'...
'%% DQN forward pass: Input→Dense(128,ReLU)→Dense(128,ReLU)→Output(6)\n'...
'h1 = max(zeros(size(W1,1),1), W1*s  + b1);\n'...
'h2 = max(zeros(size(W2,1),1), W2*h1 + b2);\n'...
'q  = W3*h2 + b3;\n'...
'\n'...
'%% Greedy action (cap at index 5 = 75% to avoid over-cooling)\n'...
'[~, ai] = max(q);\n'...
'ai = min(ai, 5);\n'...
'u_c = actions(ai);\n'...
'\n'...
'%% No cooling if already near setpoint\n'...
'if abs(Tz2 - T_SET) < 1.0; u_c = 0; end\n'...
'\n'...
'%% Rate limit (smooth operation)\n'...
'du = u_c - u_prev;\n'...
'if abs(du) > vrf_Qmax*0.35\n'...
'    u_c = u_prev + sign(du)*vrf_Qmax*0.35;\n'...
'end\n'...
'Q_cool = max(0, min(vrf_Qmax, u_c));\n'...
'u_prev = Q_cool;\n'...
'end\n'];

set_mf_code(mfb, code);
try
    % Connect signal ports
    for p = 1:length(sig_ports)
        add_line(ss,[sig_ports{p} '/1'],['RL_DQN_MF/' num2str(p)],'autorouting','on');
    end
    % Connect weight ports
    for p = 1:length(wt_ports)
        add_line(ss,[wt_ports{p} '/1'],['RL_DQN_MF/' num2str(length(sig_ports)+p)],...
            'autorouting','on');
    end
    add_line(ss,'RL_DQN_MF/1','Q_cool/1','autorouting','on');
catch; end
end

function build_hybrid_ctrl(mdl, xpos, ypos, P)
%% Hybrid controller Simulink subsystem

ss = [mdl '/Ctrl_Hybrid'];
add_block('simulink/Ports & Subsystems/Subsystem', ss,...
    'Position',[xpos ypos xpos+160 ypos+380]);
clean_subsystem(ss);

%% Inports
hyb_ports = {'T_zone1','T_zone2','T_zone3','T_outdoor',...
             'hour_of_day','season','tod_tariff','RH'};
for p = 1:length(hyb_ports)
    add_inport(ss, hyb_ports{p}, p, [20, 20+p*42]);
end
add_outport(ss, 'Q_cool', 1, [580 200]);

mfb = [ss '/Hybrid_MF'];
add_block('simulink/User-Defined Functions/MATLAB Function', mfb,...
    'Position',[80 20 540 560]);

code = [...
'function Q_cool = hybrid_ctrl(Tz1,Tz2,Tz3,T_out,h24,season,tariff,RH)\n'...
'%#codegen\n'...
'persistent hyb_integ hyb_eprev hyb_umpc_prev hyb_alpha\n'...
'persistent hyb_mpc_hist hyb_pid_hist hyb_step\n'...
'if isempty(hyb_integ)\n'...
'    hyb_integ=[0;0;0]; hyb_eprev=[0;0;0]; hyb_umpc_prev=0;\n'...
'    hyb_alpha=[0.60;0.35;0.05];\n'...
'    hyb_mpc_hist=zeros(48,1); hyb_pid_hist=zeros(48,1); hyb_step=1;\n'...
'end\n'...
'\n'...
'vrf_Qmax=11000; T_SET=24; dt=3600;\n'...
'frac=[0.316,0.474,0.210];\n'...
'Kp=[600,1000,450]; Ki=Kp./5400; Kd=Kp.*300;\n'...
'iLim=[4000,5000,2500];\n'...
'is_night=(h24>=23||h24<=6);\n'...
'SP=T_SET+2*double(is_night);\n'...
'T_now=Tz2; T_avg=(Tz1+Tz2+Tz3)/3;\n'...
'\n'...
'%% Layer 1: MPC\n'...
'Q_mpc=0;\n'...
'if T_now > SP+0.10\n'...
'    a=0.67; sum_e=0; xk=T_now;\n'...
'    T_bias=T_out+1.5;\n'...
'    for k=1:24\n'...
'        xk=a*xk+(1-a)*T_bias;\n'...
'        sum_e=sum_e+max(0,xk-SP)*power(0.93,k-1);\n'...
'    end\n'...
'    if tariff>=8.95; tod=1.35;\n'...
'    elseif tariff<=5.20; tod=0.88;\n'...
'    else; tod=1.00; end\n'...
'    Q_mpc=900*sum_e/24/tod;\n'...
'    Q_mpc=max(Q_mpc,(T_now-SP)*650);\n'...
'    if T_now>SP+0.8;  Q_mpc=max(Q_mpc,vrf_Qmax*0.15); end\n'...
'    if T_now>SP+2.0;  Q_mpc=max(Q_mpc,vrf_Qmax*0.45); end\n'...
'    if T_now>SP+3.5;  Q_mpc=max(Q_mpc,vrf_Qmax*0.75); end\n'...
'    if T_now>SP+5.0;  Q_mpc=vrf_Qmax; end\n'...
'    du=Q_mpc-hyb_umpc_prev;\n'...
'    if abs(du)>vrf_Qmax*0.28; Q_mpc=hyb_umpc_prev+sign(du)*vrf_Qmax*0.28; end\n'...
'    Q_mpc=max(vrf_Qmax*0.20,min(vrf_Qmax,Q_mpc));\n'...
'end\n'...
'hyb_umpc_prev=Q_mpc;\n'...
'\n'...
'%% Layer 2: PID\n'...
'Q_pid=0;\n'...
'Tz=[Tz1,Tz2,Tz3];\n'...
'for z=1:3\n'...
'    err=max(0,Tz(z)-SP);\n'...
'    if err<0.3; err=0; end\n'...
'    d=(err-hyb_eprev(z))/dt;\n'...
'    hyb_eprev(z)=err;\n'...
'    zm=vrf_Qmax*frac(z);\n'...
'    qz=Kp(z)*err+Ki(z)*hyb_integ(z)+Kd(z)*d;\n'...
'    qz=max(0,min(zm,qz));\n'...
'    if qz<zm*0.97&&err>0\n'...
'        hyb_integ(z)=max(0,min(iLim(z),hyb_integ(z)+err*dt));\n'...
'    elseif err==0; hyb_integ(z)=hyb_integ(z)*0.97; end\n'...
'    if qz<150; qz=0; end\n'...
'    Q_pid=Q_pid+qz;\n'...
'end\n'...
'Q_pid=min(Q_pid,vrf_Qmax*0.95);\n'...
'\n'...
'%% Layer 3: Adaptive weights\n'...
'hyb_mpc_hist(mod(hyb_step-1,48)+1)=abs(T_now-SP);\n'...
'hyb_pid_hist(mod(hyb_step-1,48)+1)=abs(T_avg-SP);\n'...
'hyb_step=hyb_step+1;\n'...
'if hyb_step>49\n'...
'    em=mean(hyb_mpc_hist)+0.1;\n'...
'    ep=mean(hyb_pid_hist)+0.1;\n'...
'    im=1/em; ip=1/ep; it=im+ip+0.1;\n'...
'    an=[im/it;ip/it;0.1/it];\n'...
'    lr=0.01;\n'...
'    hyb_alpha=(1-lr)*hyb_alpha+lr*an;\n'...
'    hyb_alpha=hyb_alpha/sum(hyb_alpha);\n'...
'    if T_now>SP+4&&mean(hyb_mpc_hist)>3\n'...
'        hyb_alpha(1)=max(0.20,hyb_alpha(1)-0.10);\n'...
'        hyb_alpha(2)=min(0.75,hyb_alpha(2)+0.10);\n'...
'        hyb_alpha(3)=1-hyb_alpha(1)-hyb_alpha(2);\n'...
'    end\n'...
'end\n'...
'\n'...
'%% Fusion\n'...
'Q_safety=min(max(0,T_avg-SP)*500,vrf_Qmax*0.40);\n'...
'Q_cool=hyb_alpha(1)*Q_mpc+hyb_alpha(2)*Q_pid+hyb_alpha(3)*Q_safety;\n'...
'Q_cool=max(0,min(vrf_Qmax,Q_cool));\n'...
'end\n'];

set_mf_code(mfb, code);

try
    lbz = hyb_ports;
    for p = 1:length(lbz)
        add_line(ss, [lbz{p} '/1'], ['Hybrid_MF/' num2str(p)], 'autorouting','on');
    end
    add_line(ss, 'Hybrid_MF/1', 'Q_cool/1', 'autorouting','on');
catch; end
end

%% ================================================================
function build_hvac_equipment(mdl, xpos, ypos)
% HVAC Equipment subsystem: VRF + IEC + DOAS energy models
ss = [mdl '/HVAC_Equipment'];
add_block('simulink/Ports & Subsystems/Subsystem', ss,...
    'Position',[xpos ypos xpos+160 ypos+280]);
clean_subsystem(ss);

for p = 1:5
    lbl = {'Q_cool_demand','T_outdoor','RH','iec_cap','doas_P'};
    add_inport(ss, lbl{p}, p, [20, 20+p*50]);
end
add_outport(ss,'Q_hvac_z1',1,[500 70]);
add_outport(ss,'Q_hvac_z2',2,[500 140]);
add_outport(ss,'Q_hvac_z3',3,[500 210]);
add_outport(ss,'P_elec',   4,[500 280]);
add_outport(ss,'COP',      5,[500 350]);
add_outport(ss,'mode',     6,[500 420]);

mfb = [ss '/HVAC_MF'];
add_block('simulink/User-Defined Functions/MATLAB Function', mfb,...
    'Position',[80 20 460 440]);

code = [...
'function [Qz1,Qz2,Qz3,P_elec,COP_out,mode_out] = hvac_equip(Q_cool,T_out,RH,iec_cap,doas_P)\n'...
'%#codegen\n'...
'vrf_Qmax=11000; COP_r=4.2; T_ref=35; a=0.035; b=0.0003;\n'...
'T_min=15; T_max=48;\n'...
'frac = [0.316, 0.474, 0.210];\n'...
'\n'...
'%% IEC assist in pre-monsoon\n'...
'Q_iec=0; P_iec=0;\n'...
'if T_out>28 && RH<50 && Q_cool>200\n'...
'    Q_iec  = iec_cap*0.5;\n'...
'    P_iec  = 150;\n'...
'end\n'...
'\n'...
'%% VRF COP\n'...
'COP = max(1.5, COP_r*(1+a*(T_ref-T_out)+b*(T_ref-T_out)^2));\n'...
'if T_out<T_min||T_out>T_max; COP=0; end\n'...
'\n'...
'%% Electrical power\n'...
'Q_tot = Q_cool + Q_iec;\n'...
'P_vrf = 0;\n'...
'if COP>0 && Q_cool>0\n'...
'    P_vrf = (Q_cool/1000)/COP;\n'...
'end\n'...
'P_elec  = P_vrf + P_iec/1000 + doas_P;\n'...
'COP_out = COP;\n'...
'\n'...
'%% Distribute to zones (negative = heat extracted)\n'...
'Qz1 = -Q_tot*frac(1);\n'...
'Qz2 = -Q_tot*frac(2);\n'...
'Qz3 = -Q_tot*frac(3);\n'...
'\n'...
'%% Mode: 1=free,2=IEC+VRF,3=VRF,4=VRF+DOAS\n'...
'if Q_cool<50\n'...
'    mode_out=1;\n'...
'elseif P_iec>0 && doas_P==0\n'...
'    mode_out=2;\n'...
'elseif doas_P>0\n'...
'    mode_out=4;\n'...
'else\n'...
'    mode_out=3;\n'...
'end\n'...
'end\n'];

set_mf_code(mfb, code);
try
    lbz = {'Q_cool_demand','T_outdoor','RH','iec_cap','doas_P'};
    for p = 1:5
        add_line(ss,[lbz{p} '/1'],['HVAC_MF/' num2str(p)],'autorouting','on');
    end
    outs = {'Q_hvac_z1','Q_hvac_z2','Q_hvac_z3','P_elec','COP','mode'};
    for p = 1:6
        add_line(ss,['HVAC_MF/' num2str(p)],[outs{p} '/1'],'autorouting','on');
    end
catch; end
end

%% ================================================================
function build_iaq_subsystem(mdl, xpos, ypos)
% IAQ subsystem: CO2 mass balance + PM2.5 with HEPA filter
ss = [mdl '/IAQ_Model'];
add_block('simulink/Ports & Subsystems/Subsystem', ss,...
    'Position',[xpos ypos xpos+160 ypos+220]);
clean_subsystem(ss);

lbl = {'T_outdoor','RH','hour_of_day','PM25_outdoor'};
for p = 1:length(lbl)
    add_inport(ss, lbl{p}, p, [20, 20+p*50]);
end
add_outport(ss,'CO2_ppm',  1,[500 80]);
add_outport(ss,'PM25_ugm3',2,[500 160]);
add_outport(ss,'DOAS_on',  3,[500 240]);

mfb = [ss '/IAQ_MF'];
add_block('simulink/User-Defined Functions/MATLAB Function', mfb,...
    'Position',[80 20 460 260]);

code = [...
'function [CO2, PM25, doas_on] = iaq_model(T_out, RH, h24, PM25_out)\n'...
'%#codegen\n'...
'persistent co2_st pm_st;\n'...
'if isempty(co2_st); co2_st=450; pm_st=15; end\n'...
'\n'...
'V=270.75; Q_inf=0.0375; Q_oa=0.06; dt=3600;\n'...
'CO2_out=420;\n'...
'occ_s=[0.9,0.9,0.9,0.9,0.9,0.8,0.6,0.3,0.2,0.1,0.1,0.2,...\n'...
'       0.5,0.5,0.3,0.2,0.3,0.5,0.7,0.8,0.9,0.9,0.9,0.9];\n'...
'ck_s=[0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0];\n'...
'h_idx = max(1,min(24,round(h24)));\n'...
'\n'...
'%% DOAS condition\n'...
'doas_on = double(RH>65 && T_out>22);\n'...
'\n'...
'%% CO2 balance\n'...
'G_co2 = 4*0.0052e-3*occ_s(h_idx);\n'...
'Q_v   = Q_inf + Q_oa*doas_on;\n'...
'dco2  = (G_co2-Q_v*(co2_st/1e6-CO2_out/1e6))/V;\n'...
'co2_st= max(CO2_out,min(3000,co2_st+dco2*dt*1e6));\n'...
'\n'...
'%% PM2.5 balance (HEPA H13)\n'...
'G_pm = 150/3600*ck_s(h_idx) + 5/3600;\n'...
'dpm  = (Q_inf*PM25_out+G_pm*V-0.3*0.9995*pm_st-0.5/3600*pm_st*V)/V;\n'...
'pm_st= max(0,min(300,pm_st+dpm*dt));\n'...
'\n'...
'CO2  = co2_st;\n'...
'PM25 = pm_st;\n'...
'end\n'];

set_mf_code(mfb, code);
try
    lbz = {'T_outdoor','RH','hour_of_day','PM25_outdoor'};
    for p = 1:5
        add_line(ss,[lbz{p} '/1'],['IAQ_MF/' num2str(p)],'autorouting','on');
    end
    add_line(ss,'IAQ_MF/1','CO2_ppm/1',  'autorouting','on');
    add_line(ss,'IAQ_MF/2','PM25_ugm3/1','autorouting','on');
    add_line(ss,'IAQ_MF/3','DOAS_on/1',  'autorouting','on');
catch; end
end

%% ================================================================
% function build_controller_mux(mdl, xpos, ypos)
% % Controller selector / MUX — allows switching between controllers
% % In the full model, all 4 run in parallel and outputs are logged
% ss = [mdl '/CtrlSelector'];
% add_block('simulink/Ports & Subsystems/Subsystem', ss,...
%     'Position',[xpos ypos xpos+120 ypos+180]);
% clean_subsystem(ss);
% 
% %% In build_controller_mux, change inport count to 6:
% for p = 1:6
%     lbl = {'Q_baseline','Q_pid','Q_mpc','Q_rl','Q_hybrid','ctrl_select'};
%     add_inport(ss, lbl{p}, p, [20, 20+p*35]);
% end
% 
% %% Update MF code to handle 5 controllers:
% code = [...
% 'function Q_out = ctrl_mux(Q_bl,Q_pid,Q_mpc,Q_rl,Q_hyb,sel)\n'...
% '%#codegen\n'...
% 'switch sel\n'...
% '    case 1; Q_out=Q_bl;\n'...
% '    case 2; Q_out=Q_pid;\n'...
% '    case 3; Q_out=Q_mpc;\n'...
% '    case 4; Q_out=Q_rl;\n'...
% '    case 5; Q_out=Q_hyb;\n'...
% '    otherwise; Q_out=0;\n'...
% 'end\n'...
% 'end\n'];
% 
% % for p = 1:5
% %     lbl = {'Q_baseline','Q_pid','Q_mpc','Q_rl','ctrl_select'};
% %     add_inport(ss, lbl{p}, p, [20, 20+p*35]);
% % end
% % add_outport(ss,'Q_active',1,[350 90]);
% % 
% % mfb = [ss '/SelMF'];
% % add_block('simulink/User-Defined Functions/MATLAB Function', mfb,...
% %     'Position',[80 20 320 200]);
% % code = [...
% % 'function Q_out = ctrl_mux(Q_bl,Q_pid,Q_mpc,Q_rl,sel)\n'...
% % '%#codegen\n'...
% % '%% sel: 1=Baseline,2=PID,3=MPC,4=RL\n'...
% % 'switch sel\n'...
% % '    case 1; Q_out = Q_bl;\n'...
% % '    case 2; Q_out = Q_pid;\n'...
% % '    case 3; Q_out = Q_mpc;\n'...
% % '    case 4; Q_out = Q_rl;\n'...
% % '    otherwise; Q_out = 0;\n'...
% % 'end\n'...
% % 'end\n'];
% set_mf_code(mfb, code);
% try
%     for p = 1:5
%         lbz = {'Q_baseline','Q_pid','Q_mpc','Q_rl','ctrl_select'};
%         add_line(ss,[lbz{p} '/1'],['SelMF/' num2str(p)],'autorouting','on');
%     end
%     add_line(ss,'SelMF/1','Q_active/1','autorouting','on');
% catch; end
% 
% % Controller select constant (default = 4 = RL; change in model to switch)
% add_block('simulink/Sources/Constant',[mdl '/CtrlSelect'],...
%     'Value','5',...   % 1=BL 2=PID 3=MPC 4=RL 5=HYBRID
%     'Position',[xpos+200 ypos+140 xpos+280 ypos+170]);
% add_annotation_block(mdl,[xpos+200 ypos+175 xpos+360 ypos+200],...
%     '1=Baseline 2=PID 3=MPC 4=RL');
% end
function build_controller_mux(mdl, xpos, ypos)

% Controller selector / MUX
ss = [mdl '/CtrlSelector'];

add_block('simulink/Ports & Subsystems/Subsystem', ss,...
    'Position',[xpos ypos xpos+420 ypos+260]);

clean_subsystem(ss);

%% INPUT PORTS
lbl = {'Q_baseline','Q_pid','Q_mpc','Q_rl','Q_hybrid','ctrl_select'};

for p = 1:6
    add_inport(ss, lbl{p}, p, [20, 20+p*35]);
end

%% OUTPUT PORT
add_outport(ss,'Q_active',1,[360 120]);

%% MATLAB FUNCTION BLOCK
mfb = [ss '/SelMF'];

add_block( ...
    'simulink/User-Defined Functions/MATLAB Function', ...
    mfb, ...
    'Position',[120 40 300 220]);

%% FUNCTION CODE
code = [ ...
'function Q_out = ctrl_mux(Q_bl,Q_pid,Q_mpc,Q_rl,Q_hyb,sel)\n' ...
'%#codegen\n' ...
'\n' ...
'switch sel\n' ...
'    case 1\n' ...
'        Q_out = Q_bl;\n' ...
'    case 2\n' ...
'        Q_out = Q_pid;\n' ...
'    case 3\n' ...
'        Q_out = Q_mpc;\n' ...
'    case 4\n' ...
'        Q_out = Q_rl;\n' ...
'    case 5\n' ...
'        Q_out = Q_hyb;\n' ...
'    otherwise\n' ...
'        Q_out = 0;\n' ...
'end\n' ...
'\n' ...
'end\n'];

set_mf_code(mfb, code);

%% CONNECT INPUTS
for p = 1:6
    add_line(ss, ...
        [lbl{p} '/1'], ...
        ['SelMF/' num2str(p)], ...
        'autorouting','on');
end

%% CONNECT OUTPUT
add_line(ss,'SelMF/1','Q_active/1','autorouting','on');

%% CONTROLLER SELECT CONSTANT
add_block( ...
    'simulink/Sources/Constant', ...
    [mdl '/CtrlSelect'], ...
    'Value','5', ...
    'Position',[xpos+480 ypos+140 xpos+560 ypos+170]);

%% ANNOTATION
add_annotation_block( ...
    mdl,...
    [xpos+480 ypos+175 xpos+700 ypos+205],...
    '1=Baseline  2=PID  3=MPC  4=RL  5=Hybrid');

end
%% ================================================================
function build_output_logger(mdl, xpos, ypos)
% To Workspace blocks + Scope for all key signals
names  = {'T_z1','T_z2','T_z3','CO2','PM25','P_elec','COP','mode'};
for k = 1:length(names)
    add_block('simulink/Sinks/To Workspace',...
        [mdl '/Log_' names{k}],...
        'VariableName', [names{k} '_out'],...
        'SaveFormat',   'Timeseries',...
        'Position',     [xpos, ypos+k*60, xpos+140, ypos+k*60+30]);
end

% Scope
add_block('simulink/Sinks/Scope',...
    [mdl '/Scope_Temps'],...
    'NumInputPorts','3',...
    'Position',[xpos, ypos+520, xpos+80, ypos+580]);
add_block('simulink/Sinks/Scope',...
    [mdl '/Scope_IAQ'],...
    'NumInputPorts','2',...
    'Position',[xpos, ypos+620, xpos+80, ypos+680]);
end

%% ================================================================
function wire_model(mdl)
% Connect all top-level blocks
zone_names = {'MasterBed','LivingDining','SecondBed'};
function safe_add_line(mdl, src, dst)
    try
        add_line(mdl, src, dst, 'autorouting','smart');
    catch e
        % silently skip already-connected or missing ports
        if ~contains(e.message,'already')
            fprintf('    Wire: %s -> %s  [skipped]\n',src,dst);
        end
    end
end

% Weather → preprocessor
safe_add_line(mdl,'FW_T_outdoor/1','WeatherPreprocessor/1');
safe_add_line(mdl,'FW_GHI/1',      'WeatherPreprocessor/2');
safe_add_line(mdl,'FW_RH/1',       'WeatherPreprocessor/3');
safe_add_line(mdl,'FW_T_wb/1',     'WeatherPreprocessor/4');

% Weather → Zones (all 3)
 for z = 1:3
    zn = "Zone" + z + "_" + zone_names{z};   % cleaner than sprintf

    safe_add_line(mdl,'WeatherPreprocessor/2', zn + "/1");
    safe_add_line(mdl,'WeatherPreprocessor/1', zn + "/2");
    safe_add_line(mdl,sprintf('FW_Qsol%d/1',z), zn + "/3");
    safe_add_line(mdl,sprintf('FW_Qint%d/1',z), zn + "/4");
end

% Inter-zone wiring (ports 6 = T_adj1, port 7 = T_adj2)
% Topology: Z1 <--R_z1z2--> Z2 <--R_z2z3--> Z3   (Z1 not adjacent to Z3)
%
% Zone 1: only neighbour is Z2.
%   Port 6 (T_adj1, R_z1z2) → T_zone2   [real coupling]
%   Port 7 (T_adj2, R_z1z2) → T_zone1   [self-feed → ΔT=0 → Q_iz2=0, harmless]
safe_add_line(mdl,'Zone2_LivingDining/1','Zone1_MasterBed/6');   % T_adj1 = T_z2
safe_add_line(mdl,'Zone1_MasterBed/1',  'Zone1_MasterBed/7');   % T_adj2 = T_z1 (self, ΔT=0)

% Zone 2: neighbours are Z1 (port 6) and Z3 (port 7)
safe_add_line(mdl,'Zone1_MasterBed/1',   'Zone2_LivingDining/6'); % T_adj1 = T_z1
safe_add_line(mdl,'Zone3_SecondBed/1',   'Zone2_LivingDining/7'); % T_adj2 = T_z3

% Zone 3: only neighbour is Z2.
%   Port 6 (T_adj1, R_z2z3) → T_zone2   [real coupling]
%   Port 7 (T_adj2, R_z2z3) → T_zone3   [self-feed → ΔT=0 → Q_iz2=0, harmless]
safe_add_line(mdl,'Zone2_LivingDining/1','Zone3_SecondBed/6');   % T_adj1 = T_z2
safe_add_line(mdl,'Zone3_SecondBed/1',  'Zone3_SecondBed/7');   % T_adj2 = T_z3 (self)

% Compute hour_of_day (0-23) from simulation clock
% Clock gives seconds → divide by 3600 → floor → mod 24 → h24 in range 1-24
add_block('simulink/Sources/Clock',[mdl '/Clock'],...
    'Position',[50 1200 90 1230]);
% Divide seconds by 3600 to get fractional hours
add_block('simulink/Math Operations/Gain',[mdl '/Sec2Hr'],...
    'Gain','1/3600','Position',[110 1198 160 1232]);
% Floor to integer hours
add_block('simulink/Math Operations/Rounding Function',[mdl '/FloorHr'],...
    'Operator','floor','Position',[180 1198 230 1232]);
% Modulo 24 to wrap daily
add_block('simulink/Math Operations/Math Function',...
    [mdl '/Mod24'],'Operator','mod','Position',[260 1198 310 1232]);
add_block('simulink/Sources/Constant',[mdl '/C_24'],...
    'Value','24','Position',[200 1250 250 1270]);
% Add 1 to convert 0-based to 1-based index
add_block('simulink/Math Operations/Sum',[mdl '/AddOne'],...
    'Inputs','++','Position',[340 1198 380 1232]);
add_block('simulink/Sources/Constant',[mdl '/C_1'],...
    'Value','1','Position',[300 1250 340 1270]);

safe_add_line(mdl,'Clock/1',   'Sec2Hr/1');
safe_add_line(mdl,'Sec2Hr/1',  'FloorHr/1');
safe_add_line(mdl,'FloorHr/1', 'Mod24/1');
safe_add_line(mdl,'C_24/1',    'Mod24/2');
safe_add_line(mdl,'Mod24/1',   'AddOne/1');
safe_add_line(mdl,'C_1/1',     'AddOne/2');

% Zone temps → controllers
% Baseline: takes T_zone_avg — feed Zone2 as representative (MF computes avg internally)
safe_add_line(mdl,'Zone2_LivingDining/1', 'Ctrl_Baseline/1');  % T_zone_avg proxy
safe_add_line(mdl,'WeatherPreprocessor/2','Ctrl_Baseline/2');
safe_add_line(mdl,'FW_RH/1',              'Ctrl_Baseline/3');

safe_add_line(mdl,'Zone1_MasterBed/1',    'Ctrl_PID/1');
safe_add_line(mdl,'Zone2_LivingDining/1', 'Ctrl_PID/2');
safe_add_line(mdl,'Zone3_SecondBed/1',    'Ctrl_PID/3');
safe_add_line(mdl,'AddOne/1',             'Ctrl_PID/4');

safe_add_line(mdl,'Zone2_LivingDining/1', 'Ctrl_MPC/1');
safe_add_line(mdl,'WeatherPreprocessor/2','Ctrl_MPC/2');
safe_add_line(mdl,'FW_tariff/1',          'Ctrl_MPC/3');
safe_add_line(mdl,'AddOne/1',             'Ctrl_MPC/4');

safe_add_line(mdl,'Zone2_LivingDining/1', 'Ctrl_RL_DQN/1');
safe_add_line(mdl,'WeatherPreprocessor/2','Ctrl_RL_DQN/2');
safe_add_line(mdl,'FW_RH/1',              'Ctrl_RL_DQN/3');
safe_add_line(mdl,'AddOne/1',             'Ctrl_RL_DQN/4');
safe_add_line(mdl,'FW_season/1',          'Ctrl_RL_DQN/5');
safe_add_line(mdl,'FW_tariff/1',          'Ctrl_RL_DQN/6');

safe_add_line(mdl,'Zone1_MasterBed/1',    'Ctrl_Hybrid/1');
safe_add_line(mdl,'Zone2_LivingDining/1', 'Ctrl_Hybrid/2');
safe_add_line(mdl,'Zone3_SecondBed/1',    'Ctrl_Hybrid/3');
safe_add_line(mdl,'WeatherPreprocessor/2','Ctrl_Hybrid/4');
safe_add_line(mdl,'AddOne/1',             'Ctrl_Hybrid/5');
safe_add_line(mdl,'FW_season/1',          'Ctrl_Hybrid/6');
safe_add_line(mdl,'FW_tariff/1',          'Ctrl_Hybrid/7');
safe_add_line(mdl,'FW_RH/1',             'Ctrl_Hybrid/8');
safe_add_line(mdl,'Ctrl_Hybrid/1',        'CtrlSelector/5');

% Weight matrix connections (ports 7-12)
wt_names_top = {'W1','W2','W3','b1','b2','b3'};
for wi = 1:6
    safe_add_line(mdl, ['WtConst_' wt_names_top{wi} '/1'],...
        ['Ctrl_RL_DQN/' num2str(6+wi)]);
end

% Controllers → selector
% safe_add_line(mdl,'Ctrl_Baseline/1','CtrlSelector/1');
% safe_add_line(mdl,'Ctrl_PID/1',     'CtrlSelector/2');
% safe_add_line(mdl,'Ctrl_MPC/1',     'CtrlSelector/3');
% safe_add_line(mdl,'Ctrl_RL_DQN/1',  'CtrlSelector/4');
% safe_add_line(mdl,'CtrlSelect/1',   'CtrlSelector/5');
%% Controllers → selector (5 inputs + 1 select = 6 total)
safe_add_line(mdl,'Ctrl_Baseline/1', 'CtrlSelector/1');
safe_add_line(mdl,'Ctrl_PID/1',      'CtrlSelector/2');
safe_add_line(mdl,'Ctrl_MPC/1',      'CtrlSelector/3');
safe_add_line(mdl,'Ctrl_RL_DQN/1',   'CtrlSelector/4');
safe_add_line(mdl,'Ctrl_Hybrid/1',   'CtrlSelector/5');
safe_add_line(mdl,'CtrlSelect/1',    'CtrlSelector/6');

% Selector → HVAC equipment
safe_add_line(mdl,'CtrlSelector/1',  'HVAC_Equipment/1');
safe_add_line(mdl,'WeatherPreprocessor/2','HVAC_Equipment/2');
safe_add_line(mdl,'FW_RH/1',         'HVAC_Equipment/3');
safe_add_line(mdl,'FW_iec_cap/1',    'HVAC_Equipment/4');
safe_add_line(mdl,'FW_doas_P/1',     'HVAC_Equipment/5');

% HVAC → Zones (Q_hvac port 5)
safe_add_line(mdl,'HVAC_Equipment/1','Zone1_MasterBed/5');
safe_add_line(mdl,'HVAC_Equipment/2','Zone2_LivingDining/5');
safe_add_line(mdl,'HVAC_Equipment/3','Zone3_SecondBed/5');

% IAQ
safe_add_line(mdl,'WeatherPreprocessor/2','IAQ_Model/1');
safe_add_line(mdl,'FW_RH/1',             'IAQ_Model/2');
safe_add_line(mdl,'AddOne/1',            'IAQ_Model/3');
safe_add_line(mdl,'FW_PM25_out/1',       'IAQ_Model/4');

% Outputs → loggers
safe_add_line(mdl,'Zone1_MasterBed/1',   'Log_T_z1/1');
safe_add_line(mdl,'Zone2_LivingDining/1','Log_T_z2/1');
safe_add_line(mdl,'Zone3_SecondBed/1',   'Log_T_z3/1');
safe_add_line(mdl,'IAQ_Model/1',         'Log_CO2/1');
safe_add_line(mdl,'IAQ_Model/2',         'Log_PM25/1');
safe_add_line(mdl,'HVAC_Equipment/4',    'Log_P_elec/1');
safe_add_line(mdl,'HVAC_Equipment/5',    'Log_COP/1');
safe_add_line(mdl,'HVAC_Equipment/6',    'Log_mode/1');

% Scope
safe_add_line(mdl,'Zone1_MasterBed/1',   'Scope_Temps/1');
safe_add_line(mdl,'Zone2_LivingDining/1','Scope_Temps/2');
safe_add_line(mdl,'Zone3_SecondBed/1',   'Scope_Temps/3');
safe_add_line(mdl,'IAQ_Model/1',         'Scope_IAQ/1');
safe_add_line(mdl,'IAQ_Model/2',         'Scope_IAQ/2');
end

%% ================================================================
%%  FIGURE GENERATION
%% ================================================================
function generate_figures(RESULTS, PuneTMY, T_db, N_SIM, T_SET, P, ...
                           tod_full, ep_R, ep_C, n_ep, ctrl_names, ctrl_ids, gf)

% colors = struct('baseline',[0.43 0.44 0.45],'pid',[0.35 0.65 1.00],...
%                 'mpc',[0.74 0.55 1.00],'rl',[0.25 0.73 0.31]);
% lw = struct('baseline',0.7,'pid',1.2,'mpc',1.2,'rl',1.8);

colors = struct('baseline',[0.43 0.44 0.45],'pid',[0.35 0.65 1.00],...
                'mpc',[0.74 0.55 1.00],'rl',[0.25 0.73 0.31],...
                'hybrid',[0.98 0.75 0.10]);   % gold for hybrid
lw = struct('baseline',0.7,'pid',1.2,'mpc',1.2,'rl',1.8,'hybrid',2.2);

season_sc = [0.22 0.48 0.73; 0.83 0.46 0.09; 0.06 0.44 0.33; 0.33 0.29 0.72];
t_month = month(PuneTMY.t_hours(1:N_SIM));
MONTHS = {'Jan','Feb','Mar','Apr','May','Jun','Jul','Aug','Sep','Oct','Nov','Dec'};

%% ── Fig 1: Zone temperatures (4-way)
figure('Name','Fig01_ZoneTemperatures','Position',[50 50 1400 700]);
subplot(3,1,1);
hold on;
for s=1:5
    idx=find(PuneTMY.season(1:N_SIM)==s);
    if ~isempty(idx)
        patch([idx(1) idx(end) idx(end) idx(1)],[10 10 50 50],...
            season_sc(s,:),'FaceAlpha',0.08,'EdgeColor','none');
    end
end
for ci=1:5
    c=ctrl_ids{ci};
    plot(1:N_SIM, RESULTS.(c).Tz2,'Color',colors.(c),'LineWidth',lw.(c));
end
plot(1:N_SIM, T_db(1:N_SIM),'Color',[0.7 0.7 0.7],'LineWidth',0.4,'LineStyle','--');
yline(26,'--r','LineWidth',0.8); yline(22,'--b','LineWidth',0.8);
yline(T_SET,'--k','LineWidth',1.2);
xlabel('Hour of year'); ylabel('Temperature (°C)');
title('Zone 2 (Living+Dining) — All 4 controllers vs outdoor');
legend([ctrl_names {'T_{outdoor}','Upper','Lower','Setpoint'}],...
    'Location','northwest','FontSize',8);
xlim([1 N_SIM]); ylim([10 50]); grid on;

subplot(3,1,2);
hold on;
for ci=1:5
    c=ctrl_ids{ci};
    plot(1:N_SIM, RESULTS.(c).Tz1,'Color',colors.(c),'LineWidth',lw.(c)*0.7);
end
yline(26,'--r'); yline(22,'--b');
xlabel('Hour of year'); ylabel('Temperature (°C)');
title('Zone 1 (Master Bedroom)'); grid on; xlim([1 N_SIM]); ylim([10 50]);
legend(ctrl_names,'Location','northwest','FontSize',8);

subplot(3,1,3);
hold on;
for ci=1:5
    c=ctrl_ids{ci};
    plot(1:N_SIM, RESULTS.(c).Tz3,'Color',colors.(c),'LineWidth',lw.(c)*0.7);
end
yline(26,'--r'); yline(22,'--b');
xlabel('Hour of year'); ylabel('Temperature (°C)');
title('Zone 3 (2nd Bedroom)'); grid on; xlim([1 N_SIM]); ylim([10 50]);
legend(ctrl_names,'Location','northwest','FontSize',8);

saveas(gcf,'Fig01_ZoneTemperatures.png');
fprintf('  Saved: Fig01_ZoneTemperatures.png\n');

%% ── Fig 2: Tracking error + RMSE
figure('Name','Fig02_TrackingError','Position',[50 50 1300 500]);
for ci=1:5
    subplot(1,5,ci);
    c=ctrl_ids{ci};
    err = RESULTS.(c).Tz2 - T_SET;
    plot(1:N_SIM, err,'Color',colors.(c),'LineWidth',0.6);
    hold on;
    yline(0,'--k'); yline(2,'--r','LineWidth',0.8); yline(-2,'--b','LineWidth',0.8);
    fill([1 N_SIM N_SIM 1],[2 2 20 20],[1 0.8 0.8],'FaceAlpha',0.15,'EdgeColor','none');
    fill([1 N_SIM N_SIM 1],[-20 -20 -2 -2],[0.8 0.8 1],'FaceAlpha',0.15,'EdgeColor','none');
    xlabel('Hour'); ylabel('Error (°C)');
    title(sprintf('%s\nRMSE=%.2f°C',ctrl_names{ci},RESULTS.(c).rmse));
    xlim([1 N_SIM]); grid on;
    text(N_SIM*0.05, -8, sprintf('RMSE\n%.2f°C', RESULTS.(c).rmse),...
        'FontSize',9,'FontWeight','bold','Color',colors.(c));
end
sgtitle('Tracking error from 24°C setpoint — all controllers');
saveas(gcf,'Fig02_TrackingError.png');
fprintf('  Saved: Fig02_TrackingError.png\n');

%% ── Fig 3: Monthly energy
figure('Name','Fig03_MonthlyEnergy','Position',[50 50 1200 500]);
me = zeros(12,4);
for ci=1:5
    c=ctrl_ids{ci};
    for m=1:12
        idx=(t_month==m);
        me(m,ci) = RESULTS.(c).E_vrf + RESULTS.(c).E_iec + RESULTS.(c).E_doas;
        % approximate monthly split by fraction
        me(m,ci) = me(m,ci) * sum(idx)/N_SIM;
    end
end
subplot(1,2,1);
bar_colors = [colors.baseline; colors.pid; colors.mpc; colors.rl; colors.hybrid];
b = bar(me,'grouped'); hold on;
for ci=1:5; b(ci).FaceColor=bar_colors(ci,:); b(ci).FaceAlpha=0.85; end
xlabel('Month'); ylabel('Energy (kWh)');
title('Monthly energy consumption (all controllers)');
xticklabels(MONTHS); legend(ctrl_names,'FontSize',8); grid on;

subplot(1,2,2);
E_components = zeros(4,3);
for ci=1:5
    c=ctrl_ids{ci};
    E_components(ci,:) = [RESULTS.(c).E_vrf, RESULTS.(c).E_iec, RESULTS.(c).E_doas];
end
b2 = bar(E_components,'stacked');
b2(1).FaceColor=[0.2 0.4 0.8]; b2(2).FaceColor=[0.1 0.7 0.4]; b2(3).FaceColor=[0.9 0.5 0.1];
xticklabels(ctrl_names); ylabel('Annual energy (kWh)');
title('Annual energy by component');
legend({'VRF','IEC','DOAS'},'FontSize',8); grid on;

sgtitle('Energy consumption analysis');
saveas(gcf,'Fig03_MonthlyEnergy.png');
fprintf('  Saved: Fig03_MonthlyEnergy.png\n');

%% ── Fig 4: Comfort distribution
figure('Name','Fig04_ComfortDistribution','Position',[50 50 1100 500]);
subplot(1,2,1);
comfort_data = zeros(3,4);
for ci=1:5
    c=ctrl_ids{ci};
    comfort_data(:,ci) = [RESULTS.(c).h_cool/N_SIM*100;
                          RESULTS.(c).h_comf/N_SIM*100;
                          RESULTS.(c).h_heat/N_SIM*100];
end
b3 = bar(comfort_data','stacked');
b3(1).FaceColor=[0.95 0.35 0.25]; b3(2).FaceColor=[0.25 0.75 0.35];
b3(3).FaceColor=[0.30 0.40 0.90];
xticklabels(ctrl_names); ylabel('% of annual hours');
title('Comfort distribution'); ylim([0 100]);
legend({'Cooling needed >26°C','Comfortable 22–26°C','Heating needed <22°C'},...
    'FontSize',8,'Location','southoutside');
grid on;
for ci=1:5
    text(ci, 102, sprintf('%.0f%%',comfort_data(2,ci)),...
        'HorizontalAlignment','center','FontSize',10,'FontWeight','bold',...
        'Color',colors.(ctrl_ids{ci}));
end

subplot(1,2,2);
% Temperature duration curve — Zone 2
ts_sorted = cell(4,1);
for ci=1:5; ts_sorted{ci} = sort(RESULTS.(ctrl_ids{ci}).Tz2,'descend'); end
hold on;
for ci=1:5
    plot(1:N_SIM, ts_sorted{ci},'Color',colors.(ctrl_ids{ci}),'LineWidth',lw.(ctrl_ids{ci}));
end
yline(26,'--r','26°C cooling'); yline(22,'--b','22°C lower');
xlabel('Hours (sorted)'); ylabel('Zone 2 temperature (°C)');
title('Temperature duration curve — Zone 2');
legend(ctrl_names,'FontSize',8); grid on;

sgtitle('Comfort analysis — 4-controller comparison');
saveas(gcf,'Fig04_ComfortDistribution.png');
fprintf('  Saved: Fig04_ComfortDistribution.png\n');

%% ── Fig 5: ToD load shifting
figure('Name','Fig05_ToDLoadShifting','Position',[50 50 1100 500]);
tod_pattern = [5.20 5.20 5.20 5.20 5.20 5.20 8.95 8.95 8.95 ...
               7.15 7.15 7.15 7.15 7.15 7.15 7.15 7.15 7.15 ...
               8.95 8.95 8.95 8.95 8.95 5.20];
subplot(1,2,1);
hrs = 0:23;
yyaxis left;
hold on;
for ci=1:5
    c=ctrl_ids{ci};
    % mean hourly VRF power
    hmean = arrayfun(@(h) mean(RESULTS.(c).E_vrf * ...
        sum(mod((1:N_SIM)-1,24)==h)/N_SIM .* ones(1,24)), 0:23);
    % use real approach
    hvals = zeros(1,24);
    for h=0:23
        idx_h = (mod((1:N_SIM)-1,24)==h);
        if any(idx_h)
            hvals(h+1) = mean(tod_full(idx_h));  % proxy
        end
    end
    % Proportional energy per hour (estimated)
    Eh = RESULTS.(c).E_vrf/N_SIM * 24;
    hvals2 = zeros(1,24);
    for h=0:23
        idx_h = (mod((1:N_SIM)-1,24)==h);
        hvals2(h+1) = sum(idx_h)/N_SIM * Eh * 1000;  % W average
    end
    plot(hrs, hvals2,'Color',colors.(c),'LineWidth',lw.(c));
end
ylabel('Mean VRF power (W)');
xlabel('Hour of day');
yyaxis right;
bar(hrs, tod_pattern,'FaceColor',[1 0.85 0.6],'FaceAlpha',0.4,'EdgeColor','none');
ylabel('ToD tariff (₹/kWh)');
ylim([4 10]);
title('Mean hourly VRF load vs ToD tariff');
legend([ctrl_names {'Tariff'}],'FontSize',8,'Location','northwest');
grid on; xlim([-0.5 23.5]);

subplot(1,2,2);
% VRF COP curve
T_test = 15:50;
COP_v = max(1.5, P.vrf_COP*(1+P.vrf_cop_a*(P.vrf_T_ref-T_test)+...
                              P.vrf_cop_b*(P.vrf_T_ref-T_test).^2));
plot(T_test, COP_v,'b-','LineWidth',2); hold on;
xline(35,'--r','ISEER 35°C','LineWidth',1.2,'LabelHorizontalAlignment','left');
xline(41,'--k','Pune peak 41°C','LineWidth',1.2);
scatter([28 35 41 44], arrayfun(@(t)max(1.5,P.vrf_COP*(1+P.vrf_cop_a*...
    (P.vrf_T_ref-t)+P.vrf_cop_b*(P.vrf_T_ref-t)^2)),[28 35 41 44]),...
    80,'r','filled');
xlabel('Outdoor temperature (°C)'); ylabel('COP');
title(sprintf('VRF COP curve  (Rated %.1f kW @ COP=%.1f)',P.vrf_Qmax/1000,P.vrf_COP));
ylim([1 7]); grid on;
text(17,6.5,sprintf('IEC active: RH<50%%, T>28°C'),'FontSize',8);

sgtitle('Time-of-day load shifting and VRF performance');
saveas(gcf,'Fig05_ToDLoadShifting.png');
fprintf('  Saved: Fig05_ToDLoadShifting.png\n');

%% ── Fig 6: IAQ — CO2 + PM2.5
figure('Name','Fig06_IAQModel','Position',[50 50 1200 500]);
subplot(1,2,1);
hold on;
for ci=1:5
    c=ctrl_ids{ci};
    plot(1:N_SIM, RESULTS.(c).CO2,'Color',colors.(c),'LineWidth',lw.(c));
end
yline(1420,'--k','ASHRAE 1420 ppm','LineWidth',1.2);
yline(420, '--g','Outdoor 420 ppm');
xlabel('Hour of year'); ylabel('CO_2 (ppm)');
title('Indoor CO_2 — DOAS activated when RH>65%, T>22°C');
legend(ctrl_names,'FontSize',8,'Location','northwest'); grid on; xlim([1 N_SIM]);
text(50, 1350, 'ASHRAE limit', 'FontSize',8,'Color','k');

subplot(1,2,2);
hold on;
for ci=1:5
    c=ctrl_ids{ci};
    plot(1:N_SIM, RESULTS.(c).PM25,'Color',colors.(c),'LineWidth',lw.(c));
end
yline(15,'--r','WHO 15 µg/m³','LineWidth',1.2);
yline(60,'--k','NAAQS 60 µg/m³');
xlabel('Hour of year'); ylabel('PM_{2.5} (µg/m³)');
title('Indoor PM_{2.5} — HEPA H13 filter active (η=99.95%)');
legend(ctrl_names,'FontSize',8,'Location','northeast'); grid on; xlim([1 N_SIM]);

sgtitle('Indoor air quality — CO_2 and PM_{2.5} annual profiles');
saveas(gcf,'Fig06_IAQModel.png');
fprintf('  Saved: Fig06_IAQModel.png\n');

% %% ── Fig 7: RL Training curve
% figure('Name','Fig07_RLTraining','Position',[50 50 1000 450]);
% subplot(1,2,1);
% window = 10;
% yyaxis left;
% plot(1:n_ep, movmean(ep_R,window),'b-','LineWidth',1.5); ylabel('Mean reward per step');
% yyaxis right;
% plot(1:n_ep, movmean(ep_C,window),'r-','LineWidth',1.5); ylabel('Comfort %');
% xlabel('Episode');
% title(sprintf('RL-DQN training curve (%d episodes)',n_ep));
% legend({'Reward (smoothed)','Comfort% (smoothed)'},'FontSize',9);
% grid on;
% 
% subplot(1,2,2);
% % Action distribution approximation
% rl_modes = RESULTS.rl.mode;
% mode_counts = arrayfun(@(m) sum(rl_modes==m), 1:4);
% mode_labels = {'Free cooling (Mode 1)','IEC+VRF (Mode 2)',...
%                'VRF only (Mode 3)','VRF+DOAS (Mode 4)'};
% pie(mode_counts + 1, mode_labels);
% title('RL-DQN: HVAC mode distribution');
% 
% sgtitle('RL-DQN training analysis');
% saveas(gcf,'Fig07_RLTraining.png');
% fprintf('  Saved: Fig07_RLTraining.png\n');
%% ── Fig 7: RL Training curve
fh7 = figure('Name','Fig07_RLTraining','Position',[50 50 1000 450],...
              'Visible','on');
subplot(1,2,1);
window = 10;
ep_vec = 1:n_ep;
yyaxis left;
plot(ep_vec, movmean(ep_R, window), 'b-', 'LineWidth', 1.5);
ylabel('Mean reward per step');
yyaxis right;
plot(ep_vec, movmean(ep_C, window), 'r-', 'LineWidth', 1.5);
ylabel('Comfort %');
xlabel('Episode');
title(sprintf('RL-DQN training curve (%d episodes)', n_ep));
legend({'Reward (smoothed)', 'Comfort% (smoothed)'}, 'FontSize', 9);
grid on;

subplot(1,2,2);
rl_modes = RESULTS.rl.mode;
mode_counts = arrayfun(@(m) sum(rl_modes==m)+1, 1:5);  % +1 avoids zero slice
mode_labels = {'Free (1)', 'IEC+VRF (2)', 'VRF (3)', 'VRF+DOAS (4)'};
pie(mode_counts, mode_labels);
title('RL-DQN: HVAC mode distribution');

sgtitle('RL-DQN training analysis');
drawnow;  % force render before save
saveas(fh7, 'Fig07_RLTraining.png');
fprintf('  Saved: Fig07_RLTraining.png\n');
%% ── Fig 8: 4-Way KPI summary (final paper figure)
figure('Name','Fig08_KPISummary','Position',[50 50 1400 700]);

% Normalised KPI bars
subplot(2,3,1);
% kpi_vals = zeros(4,5);
% for ci=1:4
%     c=ctrl_ids{ci};
%     s=RESULTS.(c);
%     kpi_vals(ci,:) = [s.E_total/RESULTS.baseline.E_total,...
%                       s.h_comf/RESULTS.baseline.h_comf,...
%                       s.h_heat/RESULTS.baseline.h_heat,...
%                       s.carbon/RESULTS.baseline.carbon,...
%                       s.rmse/RESULTS.baseline.rmse];
% end
n_ctrl = length(ctrl_ids);
kpi_vals = zeros(n_ctrl, 5);
for ci = 1:n_ctrl
    c = ctrl_ids{ci};
    s = RESULTS.(c);
    if RESULTS.baseline.h_heat == 0
        heat_ratio = 1.0;
    else
        heat_ratio = s.h_heat / RESULTS.baseline.h_heat;
    end
    kpi_vals(ci,:) = [s.E_total/RESULTS.baseline.E_total,...
                      s.h_comf/RESULTS.baseline.h_comf,...
                      heat_ratio,...
                      s.carbon/RESULTS.baseline.carbon,...
                      s.rmse/RESULTS.baseline.rmse];
end
% b4 = bar(kpi_vals, 'grouped');
% bar_colors_5 = [colors.baseline; colors.pid; colors.mpc; colors.rl;
%                 0.98 0.75 0.15];  % gold for hybrid
% for ci = 1:n_ctrl; b4(ci).FaceColor = bar_colors_5(ci,:); end
b4 = bar(kpi_vals, 'grouped');
for ci = 1:n_ctrl
    b4(ci).FaceColor = bar_colors(ci,:);
end
yline(1,'--k','Baseline'); grid on;
xticklabels({'Energy','Comfort','Heating-h','Carbon','RMSE'});
ylabel('Ratio to baseline');
title('Normalised KPIs');
legend(ctrl_names,'FontSize',8);

b4=bar(kpi_vals,'grouped');
for ci=1:5; b4(ci).FaceColor=bar_colors(ci,:); end
yline(1,'--k','Baseline'); grid on;
xticklabels({'Energy','Comfort','Heating-h','Carbon','RMSE'});
ylabel('Ratio to baseline');
title('Normalised KPIs (lower=better for energy/carbon/RMSE, higher=better for comfort)');
legend(ctrl_names,'FontSize',8);

% Energy pie (RL only)
subplot(2,3,2);
pie([RESULTS.rl.E_vrf, RESULTS.rl.E_iec, RESULTS.rl.E_doas],...
    {'VRF','IEC','DOAS'});
title(sprintf('RL-DQN energy breakdown\nTotal=%.0f kWh',RESULTS.rl.E_total));

% Carbon comparison
subplot(2,3,3);
C_vals = arrayfun(@(ci) RESULTS.(ctrl_ids{ci}).carbon, 1:5);
b5 = bar(C_vals); b5.FaceColor='flat';
for ci=1:5; b5.CData(ci,:)=bar_colors(ci,:); end
xticklabels(ctrl_names); ylabel('kgCO_2/yr');
title(sprintf('Carbon emissions (grid=%.2f kgCO₂/kWh)',gf)); grid on;
for ci=1:5
    text(ci, C_vals(ci)+20, sprintf('%.0f',C_vals(ci)),...
        'HorizontalAlignment','center','FontSize',9);
end

% RMSE comparison
subplot(2,3,4);
rmse_vals = arrayfun(@(ci) RESULTS.(ctrl_ids{ci}).rmse, 1:5);
b6 = bar(rmse_vals); b6.FaceColor='flat';
for ci=1:5; b6.CData(ci,:)=bar_colors(ci,:); end
xticklabels(ctrl_names); ylabel('RMSE (°C)');
title('Setpoint tracking RMSE — Zone 2'); grid on;
for ci=1:5
    text(ci, rmse_vals(ci)+0.05, sprintf('%.2f°C',rmse_vals(ci)),...
        'HorizontalAlignment','center','FontSize',9);
end

% Cost comparison
subplot(2,3,5);
cost_vals = arrayfun(@(ci) RESULTS.(ctrl_ids{ci}).cost/1000, 1:5);
b7 = bar(cost_vals); b7.FaceColor='flat';
for ci=1:5; b7.CData(ci,:)=bar_colors(ci,:); end
xticklabels(ctrl_names); ylabel('Electricity cost (₹ thousand/yr)');
title('Annual electricity cost'); grid on;
for ci=1:5
    text(ci, cost_vals(ci)+0.5, sprintf('₹%.1fk',cost_vals(ci)),...
        'HorizontalAlignment','center','FontSize',9);
end

% Summary text
subplot(2,3,6);
axis off;
yl=0.95;
text(0.5,yl,'SYSTEM SUMMARY','HorizontalAlignment','center',...
    'FontWeight','bold','FontSize',11);
yl=yl-0.08;
for ci=1:5
    c=ctrl_ids{ci};
    s=RESULTS.(c);
    str = sprintf('%s: E=%.0f kWh  Comfort=%.0f%%  RMSE=%.2f°C',...
        ctrl_names{ci}, s.E_total, s.h_comf/N_SIM*100, s.rmse);
    text(0.05,yl,str,'FontSize',8,'Color',bar_colors(ci,:));
    yl=yl-0.12;
end
yl=yl-0.05;
text(0.05,yl,sprintf('Building: 95m²  Zones: 3  VRF: %.0fkW',P.vrf_Qmax/1000),...
    'FontSize',8,'Color',[0.5 0.5 0.5]);
text(0.05,yl-0.1,sprintf('Climate: Pune  Sim: %d hours',N_SIM),...
    'FontSize',8,'Color',[0.5 0.5 0.5]);
text(0.05,yl-0.2,sprintf('RL: 8-state 6-action DQN  γ=%.2f',0.97),...
    'FontSize',8,'Color',[0.5 0.5 0.5]);
text(0.05,yl-0.3,sprintf('MPC: Hp=%d Hc=%d  PID: Kp=250 W/°C',P.mpc_Hp,P.mpc_Hc),...
    'FontSize',8,'Color',[0.5 0.5 0.5]);

sgtitle('Final 4-controller KPI comparison — Pune 3BHK HVAC System','FontSize',13);
saveas(gcf,'Fig08_KPISummary.png');
fprintf('  Saved: Fig08_KPISummary.png\n');
end

%% ================================================================
%%  UTILITY / HELPER FUNCTIONS
%% ================================================================
function [PuneTMY, Building, Thermal, HVAC_Equip, IAQ_params, PM25_outdoor] = ...
    generate_demo_data()
% Generates synthetic Pune climate and building data for standalone running

N = 8760;
fprintf('  Generating synthetic Pune TMY data...\n');

% Monthly profiles (Pune)
T_monthly  = [22,24,28,34,38,30,26,26,27,28,26,23];
RH_monthly = [55,48,42,35,30,87,90,88,80,60,55,52];
PM25_m     = [55,48,42,35,30,18,12,10,14,30,50,60];
mdays = [31,28,31,30,31,30,31,31,30,31,30,31];

T_db=zeros(N,1); RH=zeros(N,1); GHI=zeros(N,1);
t_hours = datetime(2024,1,1,1,0,0) + hours(0:N-1)';
season_v=zeros(N,1);
rng(1234);
for i=1:N
    doy=floor((i-1)/24);
    acc=0; m=12;
    for mm=1:12; acc=acc+mdays(mm); if doy<acc; m=mm; break; end; end
    h24=mod(i-1,24);
    T_db(i)=T_monthly(m)+6*sin((h24-14)*pi/12)+randn*1.2;
    RH(i)=max(15,min(98,RH_monthly(m)-12*sin((h24-14)*pi/12)+randn*3));
    GHI(i)=(h24>=5&&h24<=19)*max(0,900*sin((h24-5)*pi/14)*(1-0.3*(m>=6&&m<=9)));
    if m==12||m<=2; season_v(i)=1;
    elseif m<=5; season_v(i)=2;
    elseif m<=9; season_v(i)=3;
    else; season_v(i)=4; end
end

% Wet bulb
P_sat=611.657*exp(17.2694*T_db./(T_db+238.3));
P_atm=101325;
W=0.622*(RH/100.*P_sat)./(P_atm-RH/100.*P_sat);
T_wb=T_db.*atan(0.151977*sqrt(RH+8.313659))+atan(T_db+RH)-atan(RH-1.676331)+...
     0.00391838*RH.^1.5.*atan(0.023101*RH)-4.686035;

PuneTMY.T_db=T_db; PuneTMY.RH=RH; PuneTMY.GHI=GHI;
PuneTMY.T_wb=T_wb; PuneTMY.W=W; PuneTMY.season=season_v;
PuneTMY.t_hours=t_hours;
PuneTMY.time=(0:N-1)'*3600;
PuneTMY.DNI=zeros(N,1); PuneTMY.DHI=zeros(N,1);
PuneTMY.P_atm=P_atm*ones(N,1); PuneTMY.WindSpd=3*ones(N,1);

% Building
Building.geom.total_area_m2=95;  Building.geom.height_m=2.85;
Building.geom.zone1_area=30;     Building.geom.zone2_area=45;
Building.geom.zone3_area=20;     Building.geom.volume_m3=95*2.85;
Building.geom.WWR_avg=0.20;
Building.wall.U_value=0.163;     Building.wall.R_total=1/0.163;
Building.wall.rho=780; Building.wall.cp=840; Building.wall.t=0.230;
Building.wall.C_m2=780*840*0.230;
Building.roof.U_value=0.268;     Building.roof.R_total=1/0.268;
Building.roof.rho=2400; Building.roof.cp=880; Building.roof.t=0.125;
Building.roof.C_m2=2400*880*0.125;
Building.glz.U_value=2.8;        Building.glz.SHGC=0.55;
Building.glz.VT=0.70;            Building.glz.frame_frac=0.25;
Building.chajja.shade_factor=[0.15 0.18 0.25 0.35 0.40 0.45 ...
                               0.45 0.42 0.38 0.28 0.18 0.12];
Building.inf.ACH_natural=0.50;
Building.inf.Q_inf_m3s=0.50*95*2.85/3600;
Building.gains.n_occupants=4;    Building.gains.Q_person_W=90;
Building.gains.Q_lighting_Wm2=6; Building.gains.Q_equip_Wm2=4.5;
Building.gains.Q_cooking_W=2000;
Building.gains.occ_sched=[0.9,0.9,0.9,0.9,0.9,0.8,0.6,0.3,...
    0.2,0.1,0.1,0.2,0.5,0.5,0.3,0.2,0.3,0.5,0.7,0.8,0.9,0.9,0.9,0.9];
Building.gains.equip_sched=[0.2,0.1,0.1,0.1,0.1,0.2,0.5,0.7,...
    0.6,0.3,0.2,0.5,0.6,0.3,0.2,0.3,0.4,0.7,0.9,0.9,0.8,0.6,0.4,0.3];
Building.gains.light_sched=[0.1,0.1,0.1,0.1,0.1,0.3,0.6,0.3,...
    0.1,0.1,0.1,0.2,0.3,0.2,0.1,0.1,0.2,0.5,0.9,0.9,0.8,0.5,0.3,0.2];
Building.gains.cooking_sched=[0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0];

% Thermal model
rho_air=1.18; cp_air=1006;
zone_geom = struct();
zone_geom(1).area=30; zone_geom(1).ext_perimeter=11; zone_geom(1).width=6; zone_geom(1).height=2.85;
zone_geom(2).area=45; zone_geom(2).ext_perimeter=15; zone_geom(2).width=6; zone_geom(2).height=2.85;
zone_geom(3).area=20; zone_geom(3).ext_perimeter=9.2; zone_geom(3).width=5.7; zone_geom(3).height=2.85;
stone_C_m2=2200*0.005*800;

orient_f=[0.85,1.00,0.25];
chajja=Building.chajja.shade_factor;
Q_solar_zone=zeros(N,3); Q_int_8760=zeros(N,3);
zone_area_frac=[30,45,20]/95;

for z=1:3
    A_wall_g=zone_geom(z).ext_perimeter*2.85;
    zone_geom(z).A_glz=A_wall_g*0.20;
    zone_geom(z).A_wall_solid=A_wall_g-zone_geom(z).A_glz;
    zone_geom(z).A_roof=zone_geom(z).area;
    zone_geom(z).volume=zone_geom(z).area*2.85;
    C_air=rho_air*cp_air*zone_geom(z).volume;
    % zone_geom(z).C_total=C_air+stone_C_m2*zone_geom(z).area+0.3*C_air;
    % FIXED (realistic Indian RCC construction):
    C_furniture = 50000 * zone_geom(z).area / 95;  % furniture/contents
    C_rcc = 2400 * 880 * 0.150 * zone_geom(z).area; % 150mm RCC slab
    zone_geom(z).C_total = C_air + stone_C_m2*zone_geom(z).area + ...
                            C_furniture + C_rcc + 0.5*C_air;
    zone_geom(z).UA_wall=Building.wall.U_value*zone_geom(z).A_wall_solid;
    zone_geom(z).UA_roof=Building.roof.U_value*zone_geom(z).A_roof;
    zone_geom(z).UA_glz=Building.glz.U_value*zone_geom(z).A_glz;
    zone_geom(z).UA_inf=rho_air*cp_air*(Building.inf.ACH_natural/3600*zone_geom(z).volume);
    zone_geom(z).UA_total=zone_geom(z).UA_wall+zone_geom(z).UA_roof+...
                          zone_geom(z).UA_glz+zone_geom(z).UA_inf;
    zone_geom(z).tau_hours=zone_geom(z).C_total/zone_geom(z).UA_total/3600;
end

R_int_m2=(0.115/0.19)+2*(0.012/0.72)+0.240;
T_R_z1z2=R_int_m2/(6*2.85); T_R_z2z3=R_int_m2/(5.7*2.85);

T_sol_air_v=T_db+(0.70*GHI-0.90*63)/22.7;
for i=1:N
    m2=month(t_hours(i)); sf=chajja(m2);
    for z=1:3
        Q_solar_zone(i,z)=Building.glz.SHGC*zone_geom(z).A_glz*...
            (0.6*GHI(i))*(1-sf)*orient_f(z);
    end
    h24=mod(i-1,24)+1;
    Q_bg=Building.gains.n_occupants*Building.gains.Q_person_W*Building.gains.occ_sched(h24)+...
         95*Building.gains.Q_equip_Wm2*Building.gains.equip_sched(h24)+...
         95*Building.gains.Q_lighting_Wm2*Building.gains.light_sched(h24);
    Q_ck=Building.gains.Q_cooking_W*Building.gains.cooking_sched(h24);
    for z=1:3
        Q_int_8760(i,z)=Q_bg*zone_area_frac(z);
    end
    Q_int_8760(i,2)=Q_int_8760(i,2)+Q_ck;
end

Thermal.zone=zone_geom;
Thermal.R_z1z2=T_R_z1z2; Thermal.R_z2z3=T_R_z2z3;
Thermal.Q_solar=Q_solar_zone; Thermal.Q_int=Q_int_8760;
Thermal.T_sol_air=T_sol_air_v;
Thermal.zone_frac=zone_area_frac;
Thermal.total_area=95;
Thermal.TS.time=PuneTMY.time;

% HVAC equipment
HVAC_Equip.vrf.Q_rated_kW=11; HVAC_Equip.vrf.COP_rated=4.2;
HVAC_Equip.vrf.T_ref_rated=35; HVAC_Equip.vrf.cop_a=0.035;
HVAC_Equip.vrf.cop_b=0.0003; HVAC_Equip.vrf.mod_min=0.20;
HVAC_Equip.vrf.mod_max=1.10; HVAC_Equip.vrf.T_out_min=15;
HVAC_Equip.vrf.T_out_max=48;
HVAC_Equip.iec.eff_wb=0.80; HVAC_Equip.iec.P_fan_kW=0.15;
HVAC_Equip.iec.RH_enable=50; HVAC_Equip.iec.T_enable=28;
HVAC_Equip.iec.ACH=6;
HVAC_Equip.iec.capacity_kW=zeros(N,1);
rho_air=1.18; cp_air=1006; V_iec=0.35;
for i=1:N
    if RH(i)<50 && T_db(i)>28
        T_sup=T_wb(i)+2; dT=max(0,T_db(i)-T_sup);
        HVAC_Equip.iec.capacity_kW(i)=rho_air*V_iec*cp_air*dT/1000;
    end
end
HVAC_Equip.doas.V_oa_m3s=0.06; HVAC_Equip.doas.m_dot_oa=1.18*0.06;
HVAC_Equip.doas.ERV_sens_eff=0.70; HVAC_Equip.doas.ERV_lat_eff=0.65;
HVAC_Equip.doas.T_supply=14; HVAC_Equip.doas.W_supply_max=0.009;
HVAC_Equip.doas.COP_dehumid=2.8; HVAC_Equip.doas.RH_enable=65;
HVAC_Equip.doas.T_min=22;
HVAC_Equip.doas.P_elec_kW=zeros(N,1);
W_r=0.011;
for i=1:N
    if RH(i)>65 && T_db(i)>22
        W_oa=PuneTMY.W(i);
        W_post=max(W_r,W_oa-0.65*(W_oa-W_r));
        dW=max(0,W_post-0.009);
        Q_lat=HVAC_Equip.doas.m_dot_oa*2501e3*dW/1000;
        HVAC_Equip.doas.P_elec_kW(i)=Q_lat/2.8+0.02;
    end
end
HVAC_Equip.Q_inf_m3s=Building.inf.Q_inf_m3s;
tod=[5.20,5.20,5.20,5.20,5.20,5.20,8.95,8.95,8.95,...
     7.15,7.15,7.15,7.15,7.15,7.15,7.15,7.15,7.15,...
     8.95,8.95,8.95,8.95,8.95,5.20];
HVAC_Equip.tod.hourly_rate=tod'; HVAC_Equip.tod.normal_rate=7.15;
HVAC_Equip.tod.peak_rate=8.95; HVAC_Equip.tod.offpeak_rate=5.20;

% IAQ
IAQ_params.CO2_outdoor=420; IAQ_params.CO2_gen_m3s=4*0.0052e-3;
IAQ_params.eta_HEPA=0.9995; IAQ_params.lambda_dep=0.5/3600;
IAQ_params.G_PM_cook=150/3600; IAQ_params.G_PM_bg=5/3600;
IAQ_params.Q_hvac_recirc=0.30;
IAQ_params.V_total=95*2.85; IAQ_params.Q_inf=Building.inf.Q_inf_m3s;

% PM2.5 outdoor
PM25_outdoor=zeros(N,1);
PM25_hfrac=[0.75,0.70,0.68,0.68,0.72,0.85,1.20,1.35,1.25,1.05,...
    0.90,0.85,0.85,0.85,0.88,0.92,1.00,1.10,1.35,1.40,1.25,1.10,0.95,0.82];
for i=1:N
    m2=month(t_hours(i)); h24=mod(i-1,24)+1;
    PM25_outdoor(i)=PM25_m(m2)*PM25_hfrac(h24);
    if RH(i)>85 && GHI(i)<50
        PM25_outdoor(i)=PM25_outdoor(i)*0.4;
    end
end
end

%% ────────────────────────────────────────────────────────────────
%% Zone Euler integration (used in pure-MATLAB sim)
function Tz = zone_euler(Tz, T_out, T_sol, idx, Q_hvac_z, P, ...
                          Qs1, Qs2, Qs3, Qi1, Qi2, Qi3, dt)
Q_sol_arr = [Qs1(idx), Qs2(idx), Qs3(idx)];
Q_int_arr = [Qi1(idx), Qi2(idx), Qi3(idx)];
for z = 1:3
    UA_z = P.UA_tot(z);
    UA_r = P.UA_roof(z);
    C_z  = P.C(z);
    Q_iz = 0;
    if z==1
        Q_iz = (Tz(2)-Tz(1))/P.R_z1z2;
    elseif z==2
        Q_iz = (Tz(1)-Tz(2))/P.R_z1z2 + (Tz(3)-Tz(2))/P.R_z2z3;
    else
        Q_iz = (Tz(2)-Tz(3))/P.R_z2z3;
    end
    Q_net = (UA_z-UA_r)*(T_out-Tz(z)) + UA_r*(T_sol-Tz(z)) + ...
            Q_sol_arr(z) + Q_int_arr(z) + Q_hvac_z(z) + Q_iz;
    Tz(z) = max(8, min(55, Tz(z) + Q_net/C_z * dt));
end
end

%% RL helpers
function s = rl_state(Tz2, T_out, RH, h24, sea, tariff, T_SET)
s = [(Tz2-T_SET)/8; (T_out-28)/12; (RH-60)/25;
     sin(2*pi*h24/24); cos(2*pi*h24/24);
     (sea-2.5)/1.5; (tariff-7.15)/1.8;
     (Tz2-T_SET)/6];
end

% function r = rl_reward(Tz2, u_cool, tariff, T_SET, Q_max)
% err = Tz2 - T_SET;
% r   = -10*err^2 - 0.8*(u_cool/1000)*tariff;
% if Tz2 < 22; r = r - 5*(22-Tz2)^2; end
% if Tz2 > 26; r = r - 10*(Tz2-26)^2; end
% if Tz2 > 26 && u_cool > 0;  r = r + 5; end
% if Tz2 > 26 && u_cool == 0; r = r - 20; end
% if Tz2 < 22 && u_cool > 0;  r = r - 15; end
% r = r - 0.02*(u_cool/Q_max)^2;
% if abs(Tz2-T_SET) < 1.0; r = r + 10; end
% r = r / 50;
% end

function r = rl_reward(Tz2, u_cool, tariff, T_SET, Q_max)
% Shaped reward: strong comfort band, moderate energy, no over-cooling penalty
err = Tz2 - T_SET;

% Comfort: Gaussian-shaped peak at setpoint
r_comfort = 15 * exp(-0.5 * (err/1.5)^2);

% Hard penalties outside comfort band
if Tz2 > 26
    r_comfort = r_comfort - 8 * (Tz2 - 26)^2;
end
if Tz2 < 22
    r_comfort = r_comfort - 5 * (22 - Tz2)^2;
end

% Energy cost (mild — comfort dominates)
r_energy = -0.0004 * (u_cool / 1000) * tariff;

% Unnecessary cooling penalty (zone already cold)
if Tz2 < 23 && u_cool > Q_max * 0.3
    r_energy = r_energy - 3;
end

% Action taken when needed (encourage response)
if Tz2 > 26 && u_cool > Q_max * 0.2
    r_comfort = r_comfort + 3;
end

r = (r_comfort + r_energy) / 20;
end

function Q = nn_forward(W, b_nn, x)
a = x(:);
for l = 1:length(W)
    z = W{l}*a + b_nn{l};
    if l < length(W); a = max(0,z); else; a = z; end
end
Q = a;
end

function [W, b_nn] = nn_sgd_step(W, b_nn, x, ai, target, lr, bs)
n_lay = length(W);
acts = cell(n_lay+1,1); zts = cell(n_lay,1);
acts{1} = x(:);
for l = 1:n_lay
    zts{l} = W{l}*acts{l} + b_nn{l};
    if l < n_lay; acts{l+1} = max(0,zts{l}); else; acts{l+1} = zts{l}; end
end
n_out = size(W{n_lay},1);
dL = zeros(n_out,1);
dL(ai) = acts{end}(ai) - target;
for l = n_lay:-1:1
    dW = dL * acts{l}';
    db = dL;
    W{l}   = W{l}   - lr/bs * dW;
    b_nn{l}= b_nn{l}- lr/bs * db;
    if l > 1
        dL = (W{l}'*dL) .* (zts{l-1} > 0);
    end
end
end

%% Misc helpers
function v = clamp(v,lo,hi); v=max(lo,min(hi,v)); end

function m = mode_from_T_RH(T_out, RH, T_avg, SP)
if T_out > 35;           m = 3;
elseif T_out > 28 && RH < 50; m = 2;
elseif RH > 65 && T_out > 22; m = 4;
elseif T_avg > SP + 0.5; m = 3;
else;                    m = 1;
end
end

function f = occ_sched(h24)
s = [0.9,0.9,0.9,0.9,0.9,0.8,0.6,0.3,0.2,0.1,0.1,0.2,...
     0.5,0.5,0.3,0.2,0.3,0.5,0.7,0.8,0.9,0.9,0.9,0.9];
f = s(max(1,min(24,round(h24))));
end

function f = cook_sched(h24)
s = [0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0];
f = s(max(1,min(24,round(h24))));
end

function assign_mws(mws, name, val); assignin(mws, name, val); end

function add_inport(ss, name, port, pos)
add_block('simulink/Ports & Subsystems/In1',[ss '/' name],...
    'Port',num2str(port),'Position',[pos(1) pos(2) pos(1)+30 pos(2)+20]);
end

function add_outport(ss, name, port, pos)
add_block('simulink/Ports & Subsystems/Out1',[ss '/' name],...
    'Port',num2str(port),'Position',[pos(1) pos(2) pos(1)+30 pos(2)+20]);
end

function clean_subsystem(ss)
lines = find_system(ss,'SearchDepth',1,'FindAll','on','Type','line');
for d=1:length(lines); try; delete_line(lines(d)); catch; end; end
blks = find_system(ss,'SearchDepth',1,'Type','block');
for d=1:length(blks)
    if ~strcmp(blks{d},ss); try; delete_block(blks{d}); catch; end; end
end
end

function set_mf_code(mfb, code_str)
try
    rt = sfroot();
    parts = strsplit(mfb,'/');
    mdl_name = parts{1};
    mdl_obj = rt.find('-isa','Simulink.BlockDiagram','Name',mdl_name);
    if ~isempty(mdl_obj)
        chart = mdl_obj.find('-isa','Stateflow.EMChart','Path',mfb);
        if ~isempty(chart)
            chart.Script = strrep(code_str,'\n',newline);
        end
    end
catch
    % If stateflow root not accessible, skip silently
end
end

function add_annotation(mdl, text_str, pos)
try
    add_block('built-in/Note',[mdl '/Note_' num2str(randi(9999))],...
        'Position',pos,'Text',text_str,...
        'FontSize','10','FontWeight','Bold');
catch; end
end

function add_annotation_block(mdl, pos, text_str)
try
    add_block('built-in/Note',[mdl '/AnnotNote_' num2str(randi(9999))],...
        'Position',pos,'Text',text_str,'FontSize','8');
catch; end
end


%% ================================================================
%%  HYBRID CONTROLLER DIAGNOSTIC HELPER
%% ================================================================
function log_hybrid_weights(alpha, hour, T_zone, SP, trf)
%% Optional: call this during debugging to track weight evolution
if mod(hour, 24) == 0  % print once per day
    fprintf('  Hybrid weights [MPC=%.2f PID=%.2f Safety=%.2f] | T=%.1f SP=%.1f tariff=%.2f\n',...
        alpha(1), alpha(2), alpha(3), T_zone, SP, trf);
end
end

%% ================================================================
%%  HYBRID PERFORMANCE ANALYZER (call from PostSim)
%% ================================================================
function analyze_hybrid_advantage(RESULTS, N_SIM, ctrl_ids, ctrl_names)
%% Computes how much each component contributed to hybrid performance

fprintf('\n──────────────────────────────────────────\n');
fprintf('HYBRID CONTROLLER COMPONENT ANALYSIS\n');
fprintf('──────────────────────────────────────────\n');

if ~isfield(RESULTS, 'hybrid')
    fprintf('  No hybrid results found.\n');
    return;
end

R_h  = RESULTS.hybrid;
R_m  = RESULTS.mpc;
R_p  = RESULTS.pid;
R_b  = RESULTS.baseline;

fprintf('\nComfort improvement over components:\n');
fprintf('  vs Baseline:  +%.1f pp\n', (R_h.h_comf-R_b.h_comf)/N_SIM*100);
fprintf('  vs PID:       +%.1f pp\n', (R_h.h_comf-R_p.h_comf)/N_SIM*100);
fprintf('  vs MPC:       +%.1f pp\n', (R_h.h_comf-R_m.h_comf)/N_SIM*100);

fprintf('\nEnergy vs components:\n');
fprintf('  vs Baseline: %+.1f%%\n', (R_b.E_total-R_h.E_total)/R_b.E_total*100);
fprintf('  vs PID:      %+.1f%%\n', (R_p.E_total-R_h.E_total)/R_p.E_total*100);
fprintf('  vs MPC:      %+.1f%%\n', (R_m.E_total-R_h.E_total)/R_m.E_total*100);

fprintf('\nRMSE improvement:\n');
fprintf('  Hybrid: %.2f°C | MPC: %.2f°C | PID: %.2f°C\n',...
    R_h.rmse, R_m.rmse, R_p.rmse);
end

function COP = cop_vrf(T_out, P)
% VRF COP model based on outdoor temperature

% Base COP
COP = P.vrf_COP ...
      - P.vrf_cop_a * (T_out - P.vrf_T_ref) ...
      - P.vrf_cop_b * (T_out - P.vrf_T_ref).^2;

% Limit COP to realistic range
COP = max(2.0, min(6.0, COP));
end
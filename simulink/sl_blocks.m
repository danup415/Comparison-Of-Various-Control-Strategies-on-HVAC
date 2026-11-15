function varargout = sl_blocks(op, varargin)
%SL_BLOCKS  Functions behind the blocks of the block-diagram Simulink model.
%   The Simulink model Pune_HVAC_BlockDiagram.slx (built by build_pune_simulink_model.m)
%   draws the plant and controllers as separate subsystems. The linear building
%   model, the weather and disturbance inputs, the sensor noise and the signal
%   routing are native Simulink blocks. The nonlinear equipment models, the
%   moisture/CO2/PM2.5 balances and the controllers are MATLAB Function blocks
%   that call the operations below. These contain the same equations as
%   plant_step.m and the same controller functions, so the model reproduces
%   simulate.m (see check_simulink_model.m).
%
%   SL = sl_blocks('init', scenario, ctrl, seed, n_days)  set up one run
%   u  = sl_blocks('ctrl', name, k, Tmeas)                selected controller
%   [Qs, mw, P_vrf, n_on]      = sl_blocks('vrf',  Qcmd, To, W)
%   [Q_iec, P_iec, F_iec]      = sl_blocks('iec',  Qcmd, Ta, To, Twb, RHout)
%   [Q_doas, P_doas, F_d, W_d] = sl_blocks('doas', Ta, W, To, Wo, RHout)
%   [W1, CO2_1, PM1]           = sl_blocks('iaq',  W, CO2, PM, mw, F_d, W_d, F_iec, dist)
persistent P
switch op
    case 'init'
        [scenario, ctrl, seed, n_days] = varargin{:};
        cfg = scenario_config(scenario); ctx = setup_study(cfg); opts = struct();
        if strcmp(ctrl, 'rl')
            Pol = load(fullfile(cfg.out_dir, 'policies', sprintf('dqn_base_s%d.mat', seed)));
            opts.rl_policy = Pol.pol;
        end
        E = sim_init(ctx, cfg, ctrl, seed, opts);
        P = struct('E', E, 'cfg', cfg, 'pm', E.pm, 'ctrl', ctrl);
        D = E.D; n = n_days*96; t = (0:n-1)'*cfg.Ts;
        % disturbance vector (18 signals), in the order used by the Selector blocks
        dist = [D.To(1:n) D.Wout(1:n) D.Twb(1:n) D.RHout(1:n) D.Tsa_roof(1:n) ...
                D.Tsa_wall(1:n,:) D.Qsol(1:n,:) D.Qint(1:n,:) D.Glat(1:n) ...
                D.npeople(1:n) D.PMout(1:n) D.cook(1:n)];
        names = {'thermostat','pid','mpc','rl','hybrid'};
        assert(any(strcmp(names, ctrl)), 'sl_blocks:init', 'Controller %s has no block in the model', ctrl);
        SL = struct();
        SL.Ts = cfg.Ts; SL.n_days = n_days;
        SL.DIST = [t dist];
        SL.NOISE = [t E.noise(1:n,:)];
        SL.Ad = E.pm.Ad; SL.B = [E.pm.Bd E.pm.Ed];
        SL.x0 = E.s.x; SL.W0 = E.s.W; SL.CO2_0 = E.s.CO2; SL.PM0 = E.s.PM;
        SL.gs = E.pm.gain_solar; SL.gi = E.pm.gain_int;
        SL.P_filt = 0.06 * cfg.iaq.recirc_on;
        SL.CTRL = find(strcmp(names, ctrl));
        SL.ctrl_name = ctrl;
        varargout = {SL};

    case 'ctrl'
        [name, k, Tmeas] = varargin{:};
        assert(strcmp(name, P.ctrl), 'sl_blocks:ctrl', ...
               'Controller %s is active but %s was initialised', name, P.ctrl);
        obs.k = double(k); obs.Tmeas = double(Tmeas(:))';
        [u, P.E.c] = P.E.f(P.E.c, obs);
        varargout = {u(:)};

    case 'vrf'
        [Qcmd, To, W] = varargin{:};
        [Qs, mw, P_vrf, ~, n_on] = vrf_model(P.cfg.vrf, Qcmd(:)', To, W);
        varargout = {Qs(:), mw, P_vrf, n_on};

    case 'iec'
        [Qcmd, Ta, To, Twb, RHout] = varargin{:};
        rho = 1.18; cp = 1006; e = P.cfg.iec; pm = P.pm;
        Ta = Ta(:)'; Ta_m = Ta * pm.zone_frac';
        Q_iec = zeros(1,3); P_iec = 0; F_iec = 0;
        Tsup_iec = To - e.eff_wb * (To - Twb);
        if To > e.T_on && RHout < e.RH_on && sum(Qcmd) > 0 && Tsup_iec < Ta_m - e.dT_min
            F_iec = e.flow;
            Q_iec = rho*cp*F_iec*pm.zone_frac .* (Tsup_iec - Ta);
            P_iec = e.P_fan;
        end
        varargout = {Q_iec(:), P_iec, F_iec};

    case 'doas'
        [Ta, W, To, Wo, RHout] = varargin{:};
        rho = 1.18; cp = 1006; d = P.cfg.doas; pm = P.pm;
        Ta = Ta(:)'; Ta_m = Ta * pm.zone_frac';
        F_d = 0; Q_doas = zeros(1,3); P_doas = 0; W_d = Wo;
        switch d.mode
            case 'continuous', F_d = d.flow;
            case 'humidity',   if RHout > 65 && To > 22, F_d = d.flow_humid; end
        end
        if F_d > 0
            T1 = To - d.erv_sens*(To - Ta_m);
            W1 = Wo - d.erv_lat*(Wo - W);
            if W1 > d.W_sup_max && W > d.W_in_on
                T_sup = d.T_sup_coil; W_d = d.W_sup_max;
                P_coil = rho*F_d*max(0, psy('h',T1,W1) - psy('h',T_sup,W_d)) / d.COP_coil;
            else
                T_sup = T1; W_d = W1; P_coil = 0;
            end
            Q_doas = rho*cp*F_d*pm.zone_frac .* (T_sup - Ta);
            P_doas = d.P_fan + P_coil;
        end
        varargout = {Q_doas(:), P_doas, F_d, W_d};

    case 'iaq'
        [W, CO2, PM, mw, F_d, W_d, F_iec, dist] = varargin{:};
        cfg = P.cfg; q = cfg.iaq; Ts = cfg.Ts; rho = 1.18;
        Wo = dist(2); Glat = dist(15); npeople = dist(16); PMout = dist(17); cook = dist(18);
        V = P.pm.Vtot; M = rho*V;
        F_inf = cfg.bld.ACH_inf/3600 * V;
        F_out = F_inf + F_d + F_iec;
        kW_ = rho*F_out / M;
        aW  = (rho*(F_inf*Wo + F_d*W_d + F_iec*Wo) + Glat - mw) / M;
        W1 = exact1(W, aW, kW_, Ts);
        if mw > 0, W1 = max(W1, min(W, cfg.vrf.W_coil)); end
        kC = F_out / V;
        aC = (F_out*q.CO2_out + npeople*q.CO2_gen*1e6) / V;
        CO2_1 = exact1(CO2, aC, kC, Ts);
        F_rec = q.recirc_flow * q.recirc_on;
        kP = (F_out + F_rec*q.recirc_eta)/V + q.PM_dep;
        src = PMout*(F_inf*q.PM_pen + F_d*(1-cfg.doas.filter_eta) + F_iec*(1-cfg.iec.filter_eta)) ...
              + q.PM_bg + q.PM_cook*cook;
        PM1 = exact1(PM, src/V, kP, Ts);
        varargout = {W1, CO2_1, PM1};

    otherwise
        error('sl_blocks:op', 'Unknown op %s', op);
end
end

function y = exact1(y0, a, k, dt)
if k <= 0, y = y0 + a*dt; return; end
yss = a / k;
y = yss + (y0 - yss) * exp(-k*dt);
end

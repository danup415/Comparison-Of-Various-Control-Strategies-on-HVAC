function [s, lg] = plant_step(s, Qcmd, k, D, pm, cfg)
%PLANT_STEP  Advance the full plant (thermal + moisture + CO2 + PM2.5) by one step.
%   s   : state struct  x(6x1), W (kg/kg), CO2 (ppm), PM (ug/m3), iec_on
%   Qcmd: 1x3 commanded sensible cooling (kW)
%   k   : step index into the input series D
%   pm  : plant model (from build_model, possibly with mismatch factors) with fields
%         Ad, Bd, Ed, Vtot, zone_frac, gain_solar, gain_int
%   All continuous linear parts are integrated exactly over the step (no
%   explicit Euler), so the update is unconditionally stable.
Ts = cfg.Ts; rho = 1.18; cp = 1006;
To = D.To(k); Wo = D.Wout(k); Ta = s.x(1:3)'; Ta_m = Ta * pm.zone_frac';
V  = pm.Vtot; M = rho*V;

%% VRF
[Qs, mw, P_vrf, Qtot, n_on] = vrf_model(cfg.vrf, Qcmd, To, s.W);

%% IEC (supervisory economiser stage, identical rule for every controller)
e = cfg.iec; Q_iec = zeros(1,3); P_iec = 0; F_iec = 0;
Tsup_iec = To - e.eff_wb * (To - D.Twb(k));
if To > e.T_on && D.RHout(k) < e.RH_on && sum(Qcmd) > 0 && Tsup_iec < Ta_m - e.dT_min
    F_iec = e.flow;
    Q_iec = rho*cp*F_iec*pm.zone_frac .* (Tsup_iec - Ta);   % W (negative = cooling)
    P_iec = e.P_fan;
end

%% DOAS with ERV and dehumidifying coil
d = cfg.doas; F_d = 0; Q_doas = zeros(1,3); P_doas = 0; W_d = Wo;
switch d.mode
    case 'continuous', F_d = d.flow;
    case 'humidity',   if D.RHout(k) > 65 && To > 22, F_d = d.flow_humid; end
end
if F_d > 0
    T1 = To - d.erv_sens*(To - Ta_m);
    W1 = Wo - d.erv_lat*(Wo - s.W);
    if W1 > d.W_sup_max && s.W > d.W_in_on          % dehumidify only on indoor humidity demand
        T_sup = d.T_sup_coil; W_d = d.W_sup_max;
        P_coil = rho*F_d*max(0, psy('h',T1,W1) - psy('h',T_sup,W_d)) / d.COP_coil;  % kW
    else
        T_sup = T1; W_d = W1; P_coil = 0;
    end
    Q_doas = rho*cp*F_d*pm.zone_frac .* (T_sup - Ta);
    P_doas = d.P_fan + P_coil;
end

%% Thermal (exact ZOH)
dv = [To; D.Tsa_roof(k); D.Tsa_wall(k,:)'; pm.gain_solar*D.Qsol(k,:)'; ...
      pm.gain_int*D.Qint(k,:)'; (Q_iec + Q_doas)'];
s.x = pm.Ad*s.x + pm.Bd*Qs' + pm.Ed*dv;

%% Moisture, CO2, PM2.5: dC/dt = a - kC with constant coefficients over the step -> exact update
q = cfg.iaq;
F_inf = cfg.bld.ACH_inf/3600 * V;
F_out = F_inf + F_d + F_iec;                                  % exhaust/exfiltration balances supply
% moisture
kW_ = rho*F_out / M;
aW  = (rho*(F_inf*Wo + F_d*W_d + F_iec*Wo) + D.Glat(k) - mw) / M;
W_new = exact1(s.W, aW, kW_, Ts);
if mw > 0, W_new = max(W_new, min(s.W, cfg.vrf.W_coil)); end  % coil cannot dry below ADP
s.W = W_new;
% CO2
kC = F_out / V;
aC = (F_out*q.CO2_out + D.npeople(k)*q.CO2_gen*1e6) / V;
s.CO2 = exact1(s.CO2, aC, kC, Ts);
% PM2.5
F_rec = q.recirc_flow * q.recirc_on;
kP = (F_out + F_rec*q.recirc_eta)/V + q.PM_dep;
src = D.PMout(k)*(F_inf*q.PM_pen + F_d*(1-d.filter_eta) + F_iec*(1-e.filter_eta)) ...
      + q.PM_bg + q.PM_cook*D.cook(k);
s.PM = exact1(s.PM, src/V, kP, Ts);

P_filt = 0.06 * q.recirc_on;                                  % recirculation fan (kW)
lg.Qs = Qs; lg.Qtot = Qtot; lg.P_vrf = P_vrf; lg.P_iec = P_iec; lg.P_doas = P_doas;
lg.P_filt = P_filt; lg.P = P_vrf + P_iec + P_doas + P_filt; lg.n_on = n_on;
lg.iec_on = F_iec > 0;
end

function y = exact1(y0, a, k, dt)
if k <= 0, y = y0 + a*dt; return; end
yss = a / k;
y = yss + (y0 - yss) * exp(-k*dt);
end

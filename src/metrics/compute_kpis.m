function K = compute_kpis(R, D, cfg, Ta_off)
%COMPUTE_KPIS  Annual key performance indicators from a simulation log.
%   Comfort metrics are evaluated on OCCUPIED zone-hours only, using the true
%   (noise-free) air temperature of every zone, not just Zone 2.
%   Ta_off (optional): free-floating temperatures of the same scenario. The house has
%   no heating, so cold hours are split into weather-driven (Ta_off below the band) and
%   cooling-induced (Ta below min(band, Ta_off)). Kh_total = overheating + cooling-induced.
Ts = cfg.Ts; h = Ts/3600; k = R.ks(:);
occ = D.occupied(k,:); Ta = R.Ta;
lo = D.Tlo(k,:); hi = D.Thi(k,:); sp = D.sp(k,:);
% ---- energy (kWh) and cost (INR, real hourly ToD tariff)
K.E_vrf  = sum(R.P_vrf)*h;  K.E_iec = sum(R.P_iec)*h;
K.E_doas = sum(R.P_doas)*h; K.E_filt = sum(R.P_filt)*h;
K.E_total = sum(R.P)*h;
K.E_spec  = K.E_total / sum(cfg.bld.area);
K.cost    = sum(R.P .* D.tariff(k))*h;
K.cost_vrf= sum(R.P_vrf .* D.tariff(k))*h;
K.cost_flat = K.E_total * 7.15;
K.carbon  = K.E_total * cfg.grid_factor;
peak = D.tariff(k) == max(cfg.tariff);
K.peak_share = sum(R.P_vrf(peak)) / max(1e-9, sum(R.P_vrf));   % share of VRF energy in peak-tariff hours
K.P_peak  = max(R.P);                                            % kW, 15-min peak demand
K.P_peak_mean = mean(R.P(peak));
% ---- comfort (occupied zone-hours)
over  = max(0, Ta - hi) .* occ;  under = max(0, lo - Ta) .* occ;
K.Kh_over  = sum(over(:))*h;     K.Kh_under = sum(under(:))*h;
if nargin < 4 || isempty(Ta_off), Ta_off = inf(size(Ta)); end
under_hvac = max(0, min(lo, Ta_off) - Ta) .* occ;
K.Kh_under_hvac = sum(under_hvac(:))*h;
K.Kh_total = K.Kh_over + K.Kh_under_hvac;
acc = (Ta <= hi) & (Ta >= min(lo, Ta_off) - 0.1);
K.accept_pct = 100 * sum(acc(:) & occ(:)) / sum(occ(:));
inb = (Ta >= lo) & (Ta <= hi);
K.comfort_pct = 100 * sum(inb(:) & occ(:)) / sum(occ(:));
K.comfort_pct_zone = 100 * sum(inb & occ) ./ sum(occ);
e = (Ta - sp) .* occ;
K.rmse_occ = sqrt(sum(e(:).^2) / sum(occ(:)));
K.max_over = max(over(:));
% ---- humidity
RHin = psy('rh', Ta*cfg.bld.area'/sum(cfg.bld.area), R.W, D.Patm(k));
anyocc = any(occ, 2);
K.RH_mean = mean(RHin);
K.h_RH_over = sum((RHin > cfg.comfort.RH_max) & anyocc)*h;
% ---- IAQ
K.CO2_mean = mean(R.CO2); K.CO2_p95 = prctile_(R.CO2, 95);
K.h_CO2_1000 = sum((R.CO2 > 1000) & anyocc)*h;
K.PM_mean = mean(R.PM);
nd = floor(numel(k)/D.spd);
PMd = mean(reshape(R.PM(1:nd*D.spd), D.spd, nd), 1);
K.d_PM_15 = sum(PMd > 15); K.d_PM_60 = sum(PMd > 60);
% ---- equipment behaviour
on = sum(R.Qs, 2) > 1e-6;
K.starts = sum(diff([0; on]) > 0);
K.runtime_h = sum(on)*h;
K.iec_h = sum(R.iec_on)*h;
K.ctrl_ms = 1000*R.ctrl_s_per_step;
% ---- daily series for paired statistics
K.daily_cost = sum(reshape(R.P(1:nd*D.spd) .* D.tariff(k(1:nd*D.spd)), D.spd, nd), 1)'*h;
dk = sum(over + under_hvac, 2);
K.daily_Kh = sum(reshape(dk(1:nd*D.spd), D.spd, nd), 1)'*h;
K.daily_E = sum(reshape(R.P(1:nd*D.spd), D.spd, nd), 1)'*h;
end

function p = prctile_(x, q)
x = sort(x(:)); n = numel(x);
p = x(max(1, min(n, round(q/100*n))));
end

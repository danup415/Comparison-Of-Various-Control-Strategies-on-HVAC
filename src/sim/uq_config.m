function [cfg, x, names] = uq_config(i)
%UQ_CONFIG  Configuration for uncertainty-analysis sample i (Latin hypercube, fixed seed).
%   Every input that default_config.m tags [ASSUMED] is varied over a wide, physically
%   plausible range. Both the plant and the controllers' nominal model use the sampled
%   building (the plant/model mismatch factors still apply on top).
names = {'U_wall','U_roof','U_glz','SHGC','ACH_inf','COP_rated','PLR_curve_w', ...
         'n_people','gain_mult','mass_mult','cook_to_Z2'};
lo = [0.50 0.60 2.8 0.40 0.30 3.0 0 2 0.5 0.5 0.1];
hi = [2.10 3.20 5.8 0.80 1.00 4.5 1 6 1.5 1.5 0.5];
N = 40;
rs = rng(); rng(2026, 'twister');
X = zeros(N, numel(lo));
for j = 1:numel(lo)                                     % Latin hypercube
    X(:,j) = (randperm(N)' - rand(N,1)) / N;
end
rng(rs);
x = lo + X(i,:) .* (hi - lo);
cfg = default_config();
b = cfg.bld;
b.U_wall = x(1); b.U_roof = x(2); b.U_glz = x(3); b.SHGC = x(4); b.ACH_inf = x(5);
b.kappa_roof = x(10)*b.kappa_roof; b.kappa_floor = x(10)*b.kappa_floor; b.kappa_wall = x(10)*b.kappa_wall;
cfg.bld = b;
cfg.vrf.COP_rated = x(6);
cfg.vrf.eirPLR = (1 - x(7))*[0.08 0.62 0.30] + x(7)*[0.20 0.40 0.40];   % inverter -> poorer part-load curve
o = cfg.occ; f = x(8)/4;
o.occ_weekday = f*o.occ_weekday; o.occ_weekend = f*o.occ_weekend;
o.light_Wm2 = x(9)*o.light_Wm2; o.equip_Wm2 = x(9)*o.equip_Wm2; o.cook_to_Z2 = x(11);
cfg.occ = o;
cfg.scenario = sprintf('uq%d', i);
end

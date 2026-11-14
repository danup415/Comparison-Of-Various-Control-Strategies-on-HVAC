function run_tests()
%RUN_TESTS  Numerical and behavioural checks for the HVAC study (MATLAB or GNU Octave).
%   Run from the project root:  setup_paths; run_tests
cfg = default_config(); ctx = setup_study(cfg); D = ctx.D; m = ctx.nm;
np = 0; nf = 0;
    function check(cond, msg)
        if cond, np = np + 1; fprintf('  PASS  %s\n', msg);
        else,    nf = nf + 1; fprintf('  FAIL  %s\n', msg); end
    end
fprintf('Running tests...\n');

% 1 exact ZOH discretisation vs fine RK4 integration of the continuous model
x0 = [26 27 25 28 28 27]'; u = [1 2 0.5]'; d = [35; 45; 40; 38; 39; 300; 500; 200; 150; 400; 100; 0; 0; 0];
xd = x0; for k = 1:96, xd = m.Ad*xd + m.Bd*u + m.Ed*d; end
f = @(x) m.A*x + m.B*u + m.E*d; xc = x0; h = 10;
for k = 1:(96*cfg.Ts/h)
    k1 = f(xc); k2 = f(xc + h/2*k1); k3 = f(xc + h/2*k2); k4 = f(xc + h*k3);
    xc = xc + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
check(max(abs(xd - xc)) < 1e-6, sprintf('ZOH discretisation matches RK4 over 24 h (err %.1e K)', max(abs(xd - xc))));

% 2 steady state equals analytic solution of A x + B u + E d = 0
xs = -m.A \ (m.B*u + m.E*d); xd = x0; for k = 1:96*60, xd = m.Ad*xd + m.Bd*u + m.Ed*d; end
check(max(abs(xd - xs)) < 1e-4, 'Discrete model converges to analytic steady state');

% 3 stability: all discrete eigenvalues inside unit circle; time constants physical
check(all(abs(eig(m.Ad)) < 1), 'Discrete model is stable');
check(max(m.tau_h) > 10 && max(m.tau_h) < 200, sprintf('Slow (mass) time constant %.1f h is physical', max(m.tau_h)));

% 4 exact first-order IAQ update has no overshoot even when k*dt >> 2
s = struct('x', x0, 'W', 0.012, 'CO2', 600, 'PM', 200);
cfg2 = cfg; cfg2.iaq.recirc_flow = 3.0;                      % extreme removal: k*dt ~ 40
P = zeros(8,1); for j = 1:8, [s] = plant_step(s, [0 0 0], 100 + j, D, ctx.pm, cfg2); P(j) = s.PM; end
check(all(P >= 0) && all(diff(P(1:3)) <= 0), 'PM2.5 update is stable/monotone for k*dt >> 2 (old Euler oscillated)');

% 5 CO2 steady state matches analytic value
cfgc = cfg; cfgc.doas.mode = 'off';
F = cfg.bld.ACH_inf/3600*ctx.pm.Vtot; Css = cfg.iaq.CO2_out + 4*cfg.iaq.CO2_gen*1e6/F;
s.CO2 = 420; Dc = D; Dc.npeople(:) = 4;
for j = 1:400, s = plant_step(s, [0 0 0], 1000, Dc, ctx.pm, cfgc); end
check(abs(s.CO2 - Css) < 1, sprintf('CO2 reaches analytic steady state %.0f ppm', Css));

% 6 VRF: off -> no power; outside operating range -> no cooling AND no power (no free cooling)
[Qs, ~, Pv] = vrf_model(cfg.vrf, [0 0 0], 35, 0.012);
check(all(Qs == 0) && Pv == 0, 'VRF off gives zero power');
[Qs, ~, Pv] = vrf_model(cfg.vrf, [2 2 2], 52, 0.012);
check(all(Qs == 0) && Pv == 0, 'VRF outside operating range delivers no cooling (old code cooled for free)');
[Qs, ~, Pv, Qt] = vrf_model(cfg.vrf, cfg.vrf.Qiu_max, 35, 0.005);
check(abs(Qt - cfg.vrf.Qcap_rated) < 1e-9 && abs((Pv - 3*cfg.vrf.P_iu_fan) - cfg.vrf.Qcap_rated/cfg.vrf.COP_rated) < 1e-9, ...
      'VRF full load at 35 C reproduces rated COP');
[~, ~, Pa] = vrf_model(cfg.vrf, [1 1 1], 30, 0.005); [~, ~, Pb] = vrf_model(cfg.vrf, [1 1 1], 40, 0.005);
check(Pb > Pa, 'VRF power rises with outdoor temperature');

% 7 DQN feature vector and network shapes
S = rl_features(D, 500, [26 25 27], [0 0.5 1], 0);
net = nn_lib('init', [15 64 64 5]); Q = nn_lib('fwd', net, S);
check(isequal(size(S), [15 3]) && isequal(size(Q), [5 3]), 'DQN features 15x3, Q-values 5x3');
% finite-difference gradient check
X = randn(15, 4); dQ = randn(5, 4); [~, cache] = nn_lib('fwd', net, X); g = nn_lib('bwd', net, cache, dQ);
e = 1e-6; n2 = net; n2.W{1}(3,4) = n2.W{1}(3,4) + e; Q2 = nn_lib('fwd', n2, X); Q1 = nn_lib('fwd', net, X);
fd = sum(sum(dQ .* (Q2 - Q1))) / e;
check(abs(fd - g.W{1}(3,4)) < 1e-4*max(1, abs(fd)), 'MLP back-propagation matches finite differences');

% 8 controllers respect actuator limits and the MPC is deterministic for a fixed seed
kr = [96*130+1, 96*132];
umax = cfg.vrf.Qiu_max*cfg.vrf.SHR;
for c = {'thermostat', 'pid', 'mpc', 'hybrid'}
    R = simulate(ctx, cfg, c{1}, 3, struct('k_range', kr));
    check(all(R.Qcmd(:) >= -1e-9) && all(all(R.Qcmd <= repmat(umax, size(R.Qcmd,1), 1) + 1e-9)), ...
          sprintf('%s commands within [0, Umax]', c{1}));
    if strcmp(c{1}, 'mpc')
        R2 = simulate(ctx, cfg, 'mpc', 3, struct('k_range', kr));
        check(isequal(R.Ta, R2.Ta), 'Simulation is bit-reproducible for a fixed seed');
    end
end

% 9 weather is the real EPW (not synthetic): peak dry-bulb near 41 C, warmest hour mid-afternoon
[~, hpk] = max(arrayfun(@(hh) mean(D.To(D.hod == hh)), 0:23));
check(abs(max(D.To) - 41) < 0.5 && ismember(hpk-1, 13:16), sprintf('EPW weather used (max %.1f C, warmest hour %d:00)', max(D.To), hpk-1));

% 10 paired statistics: identical series -> zero difference, p = 1
P = paired_stats(ones(30,1), ones(30,1));
check(P.diff == 0 && P.p == 1, 'Paired statistics sanity');

fprintf('\n%d passed, %d failed\n', np, nf);
if nf > 0, error('run_tests:failed', '%d test(s) failed', nf); end
end

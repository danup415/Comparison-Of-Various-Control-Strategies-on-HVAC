function R = simulate(ctx, cfg, name, seed, opts)
%SIMULATE  Closed-loop simulation of one controller over the configured period.
%   R = simulate(ctx, cfg, name, seed)  name in {'thermostat','pid','mpc','rl','hybrid'}
%   opts.rl_policy : trained DQN (required for 'rl')
%   opts.k_range   : optional [k_start k_end] sub-range (used by RL validation)
%   opts.D         : optional perturbed input series (RL validation)
%   Logs the TRUE plant states (controllers only see noisy measurements).
if nargin < 5, opts = struct(); end
E = sim_init(ctx, cfg, name, seed, opts);
if isfield(opts, 'k_range'), ks = opts.k_range(1):opts.k_range(2); else, ks = 1:E.D.N; end
n = numel(ks);
L.Ta = zeros(n,3); L.Tm = zeros(n,3); L.Qs = zeros(n,3); L.Qcmd = zeros(n,3);
L.P = zeros(n,1); L.P_vrf = zeros(n,1); L.P_iec = zeros(n,1); L.P_doas = zeros(n,1);
L.P_filt = zeros(n,1); L.W = zeros(n,1); L.CO2 = zeros(n,1); L.PM = zeros(n,1);
L.n_on = zeros(n,1); L.iec_on = false(n,1);
t0 = tic;
for i = 1:n
    [E, u, lg] = sim_step(E, ks(i));
    s = E.s;
    L.Qcmd(i,:) = u;
    L.Ta(i,:) = s.x(1:3)'; L.Tm(i,:) = s.x(4:6)'; L.Qs(i,:) = lg.Qs;
    L.P(i) = lg.P; L.P_vrf(i) = lg.P_vrf; L.P_iec(i) = lg.P_iec; L.P_doas(i) = lg.P_doas;
    L.P_filt(i) = lg.P_filt; L.W(i) = s.W; L.CO2(i) = s.CO2; L.PM(i) = s.PM;
    L.n_on(i) = lg.n_on; L.iec_on(i) = lg.iec_on;
end
R = L; R.ks = ks; R.name = name; R.seed = seed;
R.wall_s = toc(t0);
R.ctrl_s_per_step = NaN;
if isfield(E.c, 't_solve'), R.ctrl_s_per_step = E.c.t_solve / max(1, E.c.nsolve); end
end

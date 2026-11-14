function E = sim_init(ctx, cfg, name, seed, opts)
%SIM_INIT  Create a closed-loop simulation engine (shared by simulate.m and the Simulink wrapper).
if nargin < 5, opts = struct(); end
D = ctx.D; if isfield(opts, 'D'), D = opts.D; end
rng(seed, 'twister');
E.noise = cfg.sensor_noise_std * randn(D.N, 3);
c = struct('cfg', cfg, 'D', D, 'nm', ctx.nm, 'pm', ctx.pm, 'seed', seed);
c.fc_xi = randn(D.N, 2);
switch name
    case 'off',        f = @ctrl_off;
    case 'thermostat', f = @ctrl_thermostat;
    case 'pid',        f = @ctrl_pid;
    case 'mpc',        f = @ctrl_mpc;
    case 'hybrid',     f = @ctrl_hybrid;
    case 'rl',         f = @ctrl_rl; c.policy = opts.rl_policy;
    otherwise, error('sim_init:name', 'Unknown controller %s', name);
end
[~, c] = f(c, [], 'init');
E.f = f; E.c = c; E.D = D; E.pm = ctx.pm; E.cfg = cfg;
k0 = 1; if isfield(opts, 'k_range'), k0 = opts.k_range(1); end
Tinit = mean(D.To(max(1,k0-96):k0)) + 3;
E.s = struct('x', Tinit*ones(6,1), 'W', D.Wout(k0), 'CO2', 600, 'PM', 15);
end

function pol = rl_train(ctx, cfg, seed)
%RL_TRAIN  Train the parameter-shared double DQN on stochastically perturbed weather.
%
%   Training data : random 14-day episodes drawn from the TMY with per-episode
%                   perturbations (uniform dry-bulb shift +-dT, solar and internal-gain
%                   scaling), so the agent never trains on the exact test trajectory.
%   Validation    : greedy return on fixed perturbed fortnights (cfg.ctrl.rl.val_starts),
%                   used for checkpoint selection only. Exploration returns are never
%                   used for model selection.
%   Test          : the unperturbed TMY year (and the +3 K heatwave year) in run_all.
%   Reward (per zone, per 15-min step), scaled by 1/r_scale for learning:
%     r = -[ ToD cost share of VRF power ] - beta*[comfort violation]^2 - switch_pen*[action changed]
%   With the safety shield on, the agent's PROPOSED action is stored together with the
%   outcome of the executed (shielded) action plus a penalty when the shield overrides it
%   (Alshiekh et al., AAAI 2018), so unsafe proposals acquire low Q-values.
%   The comfort violation is overheating above the band plus COOLING-INDUCED overcooling,
%   i.e. below min(lower band, free-floating temperature). A shadow plant without cooling
%   runs alongside each episode, so weather-driven winter cold (no heating installed)
%   is not penalised - the same definition as the evaluation KPI.
rl = cfg.ctrl.rl; D0 = ctx.D; v = cfg.vrf; Ts = cfg.Ts;
rng(1000 + seed, 'twister');
nA = numel(rl.levels); nS = 15;
net = nn_lib('init', [nS rl.hidden nA]); tgt = net; opt = [];
B = rl.buffer;
bufS = zeros(nS, B); bufA = zeros(1, B); bufR = zeros(1, B); bufS2 = zeros(nS, B);
ptr = 0; cnt = 0;
umax = v.Qiu_max * v.SHR;
ep_len = rl.ep_days * D0.spd;
n_eps = rl.episodes;
total_steps = n_eps * ep_len; step = 0;
pol.levels = rl.levels;
val_hist = []; train_ret = zeros(n_eps,1);
best = -inf; best_net = net;
Dval = cell(numel(rl.val_starts),1);
Toff_val = cell(numel(rl.val_starts),1);
for iv = 1:numel(rl.val_starts)
    Dval{iv} = perturb_inputs(D0, rl.perturb, 500 + iv);    % fixed validation perturbations
    ks = (rl.val_starts(iv)-1)*D0.spd + [1, 14*D0.spd];
    rs = rng(); Ro = simulate(ctx, cfg, 'off', 1, struct('k_range', ks, 'D', Dval{iv})); rng(rs);
    Toff_val{iv} = Ro.Ta;
end
t0 = tic;
for ep = 1:n_eps
    Dp = perturb_inputs(D0, rl.perturb, []);
    k0 = randi(D0.N - ep_len - 1);
    s = struct('x', (mean(Dp.To(k0:k0+95)) + 3)*ones(6,1), 'W', Dp.Wout(k0), 'CO2', 600, 'PM', 15);
    s_off = s;
    af = zeros(1,3); ia_prev = ones(1,3); Rsum = 0;
    noise = cfg.sensor_noise_std*randn(ep_len, 3);
    fcn = randn(ep_len,1) * min(cfg.ctrl.mpc.fc_max, cfg.ctrl.mpc.fc_sigma*sqrt(3));
    Tm = s.x(1:3)' + noise(1,:);
    S = rl_features(Dp, k0, Tm, af, fcn(1));
    for j = 1:ep_len
        k = k0 + j - 1; step = step + 1;
        epsl = max(rl.eps_min, rl.eps0 - (rl.eps0 - rl.eps_min) * step / (rl.eps_frac*total_steps));
        Q = nn_lib('fwd', net, S); [~, ia] = max(Q, [], 1);
        rnd = rand(1,3) < epsl; ia(rnd) = randi(nA, 1, sum(rnd));
        ia_prop = ia;                                           % agent's proposed action (stored)
        ia = rl_shield(ia, Tm, Dp, k, rl.levels, cfg);          % executed (shielded) action
        af = rl.levels(ia);
        [s, lg] = plant_step(s, af.*umax, k, Dp, ctx.pm, cfg);
        s_off = plant_step(s_off, [0 0 0], k, Dp, ctx.pm, cfg);
        Ta = s.x(1:3)'; Toff = s_off.x(1:3)';
        share = lg.Qs / max(1e-9, sum(lg.Qs));
        cost = Dp.tariff(k) * lg.P_vrf * share * Ts/3600;
        viol = max(0, Ta - Dp.Thi(k+1,:)) + max(0, min(Dp.Tlo(k+1,:), Toff) - Ta);
        r = -(cost + rl.beta*viol.^2 + rl.switch_pen*(ia ~= ia_prev) + rl.shield_pen*(ia ~= ia_prop)) / rl.r_scale;
        ia_prev = ia; Rsum = Rsum + sum(r);
        jn = min(j+1, ep_len);
        Tm = Ta + noise(jn,:);
        S2 = rl_features(Dp, k+1, Tm, af, fcn(jn));
        for z = 1:3
            ptr = mod(ptr, B) + 1; cnt = min(cnt+1, B);
            bufS(:,ptr) = S(:,z); bufA(ptr) = ia_prop(z); bufR(ptr) = r(z); bufS2(:,ptr) = S2(:,z);
        end
        S = S2;
        if cnt >= 1000 && mod(step, rl.update_every) == 0
            id = randi(cnt, 1, rl.batch);
            [Qb, cache] = nn_lib('fwd', net, bufS(:,id));
            Qn  = nn_lib('fwd', net, bufS2(:,id)); [~, an] = max(Qn, [], 1);   % double DQN
            Qt  = nn_lib('fwd', tgt, bufS2(:,id));
            y   = bufR(id) + rl.gamma * Qt(sub2ind(size(Qt), an, 1:rl.batch));
            q   = Qb(sub2ind(size(Qb), bufA(id), 1:rl.batch));
            err = q - y; gerr = max(-1, min(1, err)) / rl.batch;               % Huber
            dQ  = zeros(size(Qb)); dQ(sub2ind(size(Qb), bufA(id), 1:rl.batch)) = gerr;
            g   = nn_lib('bwd', net, cache, dQ);
            gn  = 0; for l = 1:numel(g.W), gn = gn + sum(g.W{l}(:).^2) + sum(g.b{l}(:).^2); end
            gn  = sqrt(gn); if gn > rl.grad_clip, for l = 1:numel(g.W), g.W{l} = g.W{l}*rl.grad_clip/gn; g.b{l} = g.b{l}*rl.grad_clip/gn; end, end
            [net, opt] = nn_lib('adam', net, opt, g, rl.lr);
            tgt = nn_lib('soft', tgt, net, rl.tau);
        end
    end
    train_ret(ep) = Rsum;
    if mod(ep, rl.val_every) == 0 || ep == n_eps
        pol.net = net; vr = 0; rs = rng();
        for iv = 1:numel(rl.val_starts)
            ks = (rl.val_starts(iv)-1)*D0.spd + [1, 14*D0.spd];
            Rv = simulate(ctx, cfg, 'rl', 900 + iv, struct('rl_policy', pol, 'k_range', ks, 'D', Dval{iv}));
            vr = vr + episode_return(Rv, Dval{iv}, cfg, Toff_val{iv});
        end
        rng(rs);
        val_hist(end+1,:) = [ep vr]; %#ok<AGROW>
        if vr > best, best = vr; best_net = net; end
        fprintf('    [DQN seed %d] ep %3d/%d  eps=%.2f  train=%8.1f  val=%8.1f  (%.0f s)\n', ...
               seed, ep, n_eps, epsl, Rsum, vr, toc(t0));
    end
end
pol.net = best_net; pol.val_hist = val_hist; pol.train_ret = train_ret;
pol.train_s = toc(t0); pol.seed = seed;
end

function Dp = perturb_inputs(D, p, s)
% Episode-level weather/gain perturbation (optional fixed seed s for validation).
if ~isempty(s), rs = rng(); rng(s, 'twister'); end
dT = (2*rand - 1) * p.dT; fs = 1 + (2*rand - 1)*p.solar; fi = 1 + (2*rand - 1)*p.int;
if ~isempty(s), rng(rs); end
Dp = D;
Dp.To = D.To + dT; Dp.Tsa_roof = D.Tsa_roof + dT; Dp.Tsa_wall = D.Tsa_wall + dT;
Dp.Qsol = fs*D.Qsol; Dp.Qint = fi*D.Qint;
Dp.Twb = psy('twb', Dp.To, psy('rh', Dp.To, D.Wout, D.Patm));
Dp.RHout = psy('rh', Dp.To, D.Wout, D.Patm);
end

function G = episode_return(R, D, cfg, Toff)
k = R.ks; Ts = cfg.Ts;
cost = sum(D.tariff(k) .* R.P_vrf) * Ts/3600;
kk = min(k + 1, D.N);
viol = max(0, R.Ta - D.Thi(kk,:)) + max(0, min(D.Tlo(kk,:), Toff) - R.Ta);
G = -(cost + cfg.ctrl.rl.beta * sum(viol(:).^2)) / cfg.ctrl.rl.r_scale;
end

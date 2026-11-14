function K = run_case(scenario, ctrl, seed, keep_series)
%RUN_CASE  Run one (scenario, controller, seed) and save KPIs to results/runs/.
%   The DQN policy for a seed is trained once on the scenario that defines its
%   design (base, or the rl_levels* ablations) and re-used, unchanged, for the
%   robustness scenarios (heatwave, mass_low, mass_high, ...).
if nargin < 4, keep_series = false; end
cfg = scenario_config(scenario);
out = fullfile(cfg.out_dir, 'runs'); if ~exist(out, 'dir'), mkdir(out); end
fn = fullfile(out, sprintf('%s__%s__s%d.mat', scenario, ctrl, seed));
if exist(fn, 'file'), S = load(fn); K = S.K; return; end
ctx = setup_study(cfg);
opts = struct();
if strcmp(ctrl, 'rl')
    pol_scn = 'base'; if strncmp(scenario, 'rl_', 3), pol_scn = scenario; end
    pdir = fullfile(cfg.out_dir, 'policies'); if ~exist(pdir, 'dir'), mkdir(pdir); end
    pfn = fullfile(pdir, sprintf('dqn_%s_s%d.mat', pol_scn, seed));
    if exist(pfn, 'file')
        P = load(pfn); pol = P.pol;
    else
        cfg_t = scenario_config(pol_scn); ctx_t = setup_study(cfg_t);
        pol = rl_train(ctx_t, cfg_t, seed); save(pfn, 'pol', '-v7');
    end
    opts.rl_policy = pol;
end
fprintf('  running %s / %s / seed %d ...\n', scenario, ctrl, seed);
Ta_off = [];
if ~strcmp(ctrl, 'off')
    run_case(scenario, 'off', 1, true);
    So = load(fullfile(out, sprintf('%s__off__s1.mat', scenario))); Ta_off = double(So.TS.Ta);
end
R = simulate(ctx, cfg, ctrl, seed, opts);
K = compute_kpis(R, ctx.D, cfg, Ta_off);
K.scenario = scenario; K.ctrl = ctrl; K.seed = seed; K.wall_s = R.wall_s;
if keep_series
    single_ = @(x) single(x);
    TS = struct('Ta', single_(R.Ta), 'Tm', single_(R.Tm), 'Qs', single_(R.Qs), 'P', single_(R.P), ...
                'P_vrf', single_(R.P_vrf), 'CO2', single_(R.CO2), 'PM', single_(R.PM), 'W', single_(R.W));
    save(fn, 'K', 'TS', '-v7');
else
    save(fn, 'K', '-v7');
end
fprintf('    %s/%s/s%d: E=%.0f kWh cost=%.0f INR Kh(over+hvac-cold)=%.1f accept=%.1f%% (%.0f s)\n', ...
       scenario, ctrl, seed, K.E_total, K.cost, K.Kh_total, K.accept_pct, R.wall_s);
end
